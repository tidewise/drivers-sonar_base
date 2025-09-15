#ifndef SONAR_BASE_SONARTOIMAGELUT_HPP
#define SONAR_BASE_SONARTOIMAGELUT_HPP

#include <base/Angle.hpp>
#include <base/samples/Sonar.hpp>
#include <opencv2/core.hpp>
#include <vector>

namespace sonar_base {
    /**
     * @brief TODO DOCUMENT
     *
     */
    class SonarToImageLUT {
    public:
        SonarToImageLUT() = delete;
        SonarToImageLUT(base::samples::Sonar const& sonar, size_t window_size);
        bool hasMatchingConfiguration(base::samples::Sonar const& sonar,
            size_t window_size);
        std::vector<std::vector<cv::Point>> computeRawLUTTable() const;
        void updateImage(cv::Mat& image,
            size_t gloabal_idx,
            int value,
            size_t bin_count) const;

    private:
        std::vector<cv::Point> m_data;
        std::vector<int> m_data_index;
        size_t m_bin_count;
        size_t m_beam_count;
        base::Angle m_beam_width;
        double m_bin_duration;
        double m_speed_of_sound;
        size_t m_window_size;
        std::vector<base::Angle> m_bearings;

        static void addRawLUTEntry(std::vector<std::vector<cv::Point>>& table,
            size_t beam_idx,
            size_t bin_idx,
            size_t bin_count,
            cv::Point const& point,
            size_t beam_count);
        static size_t binPosition(cv::Point const& point2origin,
            double distance_per_pixel,
            double bin_length);
        static size_t closestBeamIdx(base::Angle const& angle,
            double angle_resolution,
            base::Angle const& initial_angle);
        static std::optional<std::pair<int, int>> beamIndexRange(cv::Point const& point,
            double half_beam_width,
            base::Angle const& initial_angle,
            double angle_step,
            std::vector<base::Angle> const& bearings);
        static void updateLUT(cv::Point const& point,
            cv::Point const& origin,
            double distance_per_pixel,
            double bin_length,
            double half_beam_width,
            std::vector<base::Angle> const& bearings,
            double step_angle,
            size_t bin_count,
            size_t beam_count,
            std::vector<std::vector<cv::Point>>& lut);
        void linearizeRawTable(std::vector<std::vector<cv::Point>> const& raw_table);
        std::pair<std::vector<cv::Point>::const_iterator,
            std::vector<cv::Point>::const_iterator>
        getPixels(int beam_idx, int bin_idx, size_t bin_count) const;
    };
}

#endif