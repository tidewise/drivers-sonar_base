#include "SonarToImageLUT.hpp"
#include <sonar_base/SonarToImageLUT.hpp>

using namespace sonar_base;
using namespace std;
using namespace cv;

SonarToImageLUT::SonarToImageLUT(base::samples::Sonar const& sonar, size_t window_size)
    : m_bin_count(sonar.bin_count)
    , m_beam_count(sonar.beam_count)
    , m_beam_width(sonar.beam_width)
    , m_bin_duration(sonar.bin_duration.toSeconds())
    , m_speed_of_sound(sonar.speed_of_sound)
    , m_window_size(window_size)
    , m_bearings(sonar.bearings)
{
    auto raw_table = computeRawLUTTable();
    linearizeRawTable(raw_table);
}

static double computeChord(double range,
    int beam_count,
    std::vector<base::Angle> const& bearings,
    base::Angle const& beam_width)
{
    auto fov = bearings.front() - bearings.back() +
               base::Angle::fromRad(abs(beam_width.getRad()));
    return abs(2 * range * sin(fov.getRad() / 2));
}

size_t SonarToImageLUT::closestBeamIdx(base::Angle const& angle,
    double angle_resolution,
    base::Angle const& initial_angle)
{
    return round(abs((angle - initial_angle).getRad()) / angle_resolution);
}

static bool insideBeam(size_t idx,
    base::Angle const& angle,
    std::vector<base::Angle> const& bearings,
    double half_beam_width)
{
    if (idx >= bearings.size()) {
        return false;
    }

    base::AngleSegment beam(bearings[idx] - base::Angle::fromRad(half_beam_width),
        2 * half_beam_width);
    return beam.isInside(angle);
}

size_t SonarToImageLUT::binPosition(cv::Point const& point2origin,
    double distance_per_pixel,
    double bin_length)
{
    auto distance_in_pixels = norm(point2origin);
    auto distance = distance_in_pixels * distance_per_pixel;
    return round(distance / bin_length);
}

std::vector<std::vector<cv::Point>> SonarToImageLUT::computeRawLUTTable()
{
    std::vector<std::vector<cv::Point>> raw_lut;
    raw_lut.resize(m_bin_count * m_beam_count);
    double half_beam_width = m_beam_width.getRad() / 2;
    double range = m_bin_duration * m_bin_count * m_speed_of_sound;
    double chord = computeChord(range, m_beam_count, m_bearings, m_beam_width);
    double bin_length = range / m_bin_count;
    double step_angle =
        ((m_bearings.back() - m_bearings.front()).getRad()) / (m_beam_count - 1);
    double distance_per_pixel = 0;
    if (range >= chord) {
        distance_per_pixel = range / m_window_size;
        m_window_width = chord / distance_per_pixel;
        m_window_height = m_window_size;
    }
    else {
        distance_per_pixel = chord / m_window_size;
        m_window_width = m_window_size;
        m_window_height = range / distance_per_pixel;
    }
    cv::Point beam_origin = cv::Point(m_window_width / 2, m_window_height);

    for (size_t x = 0; x < m_window_width; x++) {
        for (size_t y = 0; y < m_window_height; y++) {
            updateLUT(cv::Point(x, y),
                beam_origin,
                distance_per_pixel,
                bin_length,
                half_beam_width,
                m_bearings,
                step_angle,
                m_bin_count,
                m_beam_count,
                raw_lut);
        }
    }
    return raw_lut;
}

void SonarToImageLUT::updateLUT(cv::Point const& point,
    cv::Point const& origin,
    double distance_per_pixel,
    double bin_length,
    double half_beam_width,
    std::vector<base::Angle> const& bearings,
    double step_angle,
    size_t bin_count,
    size_t beam_count,
    std::vector<std::vector<cv::Point>>& lut)
{
    auto point2origin = point - origin;

    size_t bin_position = binPosition(point2origin, distance_per_pixel, bin_length);
    if (bin_position >= bin_count) {
        return;
    }
    auto beam_idx_range =
        beamIndexRange(point2origin, half_beam_width, bearings[0], step_angle, bearings);
    if (beam_idx_range.has_value()) {
        for (int idx = beam_idx_range->first; idx <= beam_idx_range->second; idx++) {
            addRawLUTEntry(lut, idx, bin_position, bin_count, point, beam_count);
        }
    }
}

std::optional<std::pair<int, int>> SonarToImageLUT::beamIndexRange(cv::Point const& point,
    double half_beam_width,
    base::Angle const& initial_angle,
    double angle_step,
    std::vector<base::Angle> const& bearings)
{
    // Change the coordinate system to NWU
    // x' = -y, y' = -x
    cv::Point point2origin_nwu(-point.y, -point.x);

    double theta_rad = atan2(point2origin_nwu.y, point2origin_nwu.x);
    auto theta = base::Angle::fromRad(theta_rad);
    int closest_beam_idx = closestBeamIdx(theta, angle_step, initial_angle);

    if (!insideBeam(closest_beam_idx, theta, bearings, half_beam_width)) {
        return std::nullopt;
    }

    int min_idx = closest_beam_idx;
    int max_idx = closest_beam_idx;
    while (insideBeam(min_idx - 1, theta, bearings, half_beam_width)) {
        min_idx--;
    }
    while (insideBeam(max_idx + 1, theta, bearings, half_beam_width)) {
        max_idx++;
    }
    if (min_idx > max_idx) {
        std::swap(min_idx, max_idx);
    }

    return std::make_pair(min_idx, max_idx);
}

void SonarToImageLUT::addRawLUTEntry(std::vector<std::vector<cv::Point>>& table,
    size_t beam_idx,
    size_t bin_idx,
    size_t bin_count,
    cv::Point const& point,
    size_t beam_count)
{
    size_t idx = beam_idx * bin_count + bin_idx;
    if (idx < bin_count * beam_count) {
        table[beam_idx * bin_count + bin_idx].push_back(point);
    }
}

void SonarToImageLUT::linearizeRawTable(std::vector<std::vector<cv::Point>> const& table)
{
    m_data_index.resize(table.size() + 1);
    m_data.clear();
    for (size_t idx = 0; idx < table.size(); idx++) {
        m_data_index[idx] = m_data.size();
        for (size_t point_id = 0; point_id < table[idx].size(); point_id++) {
            m_data.push_back(table[idx][point_id]);
        }
    }
    m_data_index[table.size()] = m_data.size();
}

void SonarToImageLUT::updateImage(cv::Mat& image,
    size_t global_idx,
    int value,
    size_t bin_count) const
{
    size_t beam_idx = global_idx / bin_count;
    size_t bin_idx = global_idx % bin_count;
    if (value < 0) {
        value = 0;
    }
    auto its = getPixels(beam_idx, bin_idx, bin_count);
    for (auto [pixel, last_pixel] = its; pixel != last_pixel; pixel++) {
        auto& current = image.at<Vec3b>(*pixel);
        // todo value is between 0 and 1, scale to 255
        auto v = std::max<int>(current[0], value);
        current = Vec3b(v, v, v);
    }
}

pair<vector<Point>::const_iterator, vector<Point>::const_iterator> SonarToImageLUT::
    getPixels(int beam_idx, int bin_idx, size_t bin_count) const
{
    int pixels_begin = m_data_index[beam_idx * bin_count + bin_idx];
    int pixels_end = m_data_index[beam_idx * bin_count + bin_idx + 1];
    return std::make_pair(m_data.cbegin() + pixels_begin, m_data.cbegin() + pixels_end);
}

static bool bearingsMatch(std::vector<base::Angle> const& new_bearings,
    std::vector<base::Angle> const& old_bearings);

bool SonarToImageLUT::hasMatchingConfiguration(base::samples::Sonar const& sonar,
    size_t window_size)
{
    if (!(sonar.bin_count == m_bin_count && sonar.beam_count == m_beam_count &&
            sonar.beam_width == m_beam_width &&
            sonar.bin_duration.toSeconds() == m_bin_duration &&
            sonar.speed_of_sound == m_speed_of_sound && window_size == m_window_size)) {
        return false;
    }
    if (!bearingsMatch(sonar.bearings, m_bearings)) {
        return false;
    }
    return true;
}

static bool bearingsMatch(std::vector<base::Angle> const& new_bearings,
    std::vector<base::Angle> const& old_bearings)
{
    if (new_bearings.size() != old_bearings.size()) {
        return false;
    }
    for (size_t i = 0; i < old_bearings.size(); i++) {
        if (!(new_bearings[i] == old_bearings[i])) {
            return false;
        }
    }
    return true;
}

size_t SonarToImageLUT::getWindowHeight()
{
    return m_window_height;
}

size_t SonarToImageLUT::getWindowWidth()
{
    return m_window_width;
}
