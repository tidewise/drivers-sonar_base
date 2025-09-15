#include "SonarToImageLUT.hpp"
#include <sonar_base/SonarToImageLUT.hpp>

using namespace sonar_base;
using namespace std;
using namespace cv;

SonarToImageLUT::SonarToImageLUT(size_t bin_count,
    size_t beam_count,
    base::Angle beam_width,
    double bin_duration,
    double speed_of_sound,
    size_t window_size,
    std::vector<base::Angle> const& bearings)
{
    auto raw_table = computeRawLUTTable(bin_count,
        beam_count,
        beam_width,
        bin_duration,
        speed_of_sound,
        window_size,
        bearings);
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

std::vector<std::vector<cv::Point>> SonarToImageLUT::computeRawLUTTable(size_t bin_count,
    size_t beam_count,
    base::Angle beam_width,
    double bin_duration,
    double speed_of_sound,
    size_t window_size,
    std::vector<base::Angle> const& bearings)
{
    std::vector<std::vector<cv::Point>> raw_lut;
    raw_lut.resize(bin_count * beam_count);
    double half_beam_width = beam_width.getRad() / 2;
    double range = bin_duration * bin_count * speed_of_sound;
    double chord = computeChord(range, beam_count, bearings, beam_width);
    double bin_length = range / bin_count;
    double step_angle =
        ((bearings.back() - bearings.front()).getRad()) / (beam_count - 1);
    
    size_t width = 0;
    size_t height = 0;
    double distance_per_pixel = 0;
    if (range >= chord) {
        distance_per_pixel = range / window_size;
        width = chord / distance_per_pixel;
        height = window_size;
    }
    else {
        distance_per_pixel = chord / window_size;
        width = window_size;
        height = range / distance_per_pixel;
    }
    cv::Point beam_origin = cv::Point(width / 2, height);

    for (size_t x = 0; x < width; x++) {
        for (size_t y = 0; y < height; y++) {
            updateLUT(cv::Point(x, y),
                beam_origin,
                distance_per_pixel,
                bin_length,
                half_beam_width,
                bearings,
                step_angle,
                bin_count,
                beam_count,
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
    std::vector<std::vector<cv::Point>>& table)
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
            addRawLUTEntry(table, idx, bin_position, bin_count, point, beam_count);
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
