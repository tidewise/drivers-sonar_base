#include <base/samples/Sonar.hpp>
#include <gtest/gtest.h>
#include <sonar_base/SonarToImageLUT.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace sonar_base;
using namespace std;

struct SonarToImageLUTTest : public ::testing::Test {
    size_t window_size = 500;
    base::samples::Sonar sonar;
};

TEST_F(SonarToImageLUTTest,
    it_renders_the_sonar_image_when_the_beam_width_is_smaller_than_the_step_angle)
{
    sonar.bin_duration = base::Time::fromSeconds(1);
    sonar.speed_of_sound = 1;
    sonar.bin_count = 10;
    sonar.beam_count = 512 / 32;
    sonar.setRegularBeamBearings(base::Angle::fromDeg(-65),
        base::Angle::fromDeg(0.25390625 * 32));
    sonar.beam_width = base::Angle::fromDeg(0.2 * 32);
    size_t bins_size = sonar.beam_count * sonar.bin_count;
    sonar.bins.resize(bins_size, 1);

    SonarToImageLUT lut = SonarToImageLUT(sonar, window_size);

    size_t window_height = lut.getWindowHeight();
    size_t window_width = lut.getWindowWidth();

    cv::Mat img = cv::Mat::zeros(window_height, window_width, CV_8UC3);

    for (size_t i = 0; i < sonar.bins.size(); i++) {
        lut.updateImage(img, i, sonar.bins[i], sonar.bin_count);
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY); //
    cv::Mat gray_scaled;
    gray.convertTo(gray_scaled, CV_8UC1, 255);

    auto expected_img = cv::imread(PROJECT_DIR + std::string("data/fan2.png"),
        cv::ImreadModes::IMREAD_GRAYSCALE);

    ASSERT_EQ(cv::Scalar(0), cv::sum(expected_img != gray_scaled));
}

TEST_F(SonarToImageLUTTest,
    it_renders_the_sonar_image_when_the_beam_width_is_equal_the_step_angle)
{
    sonar.bin_duration = base::Time::fromSeconds(1);
    sonar.speed_of_sound = 1;
    sonar.bin_count = 10;
    sonar.beam_count = 512;
    sonar.setRegularBeamBearings(base::Angle::fromDeg(-65),
        base::Angle::fromDeg(0.25390625));
    sonar.beam_width = base::Angle::fromDeg(0.25390625);
    size_t bins_size = sonar.beam_count * sonar.bin_count;
    sonar.bins.resize(bins_size, 1);

    SonarToImageLUT lut = SonarToImageLUT(sonar, window_size);

    size_t window_height = lut.getWindowHeight();
    size_t window_width = lut.getWindowWidth();

    cv::Mat img = cv::Mat::zeros(window_height, window_width, CV_8UC3);

    for (size_t i = 0; i < sonar.bins.size(); i++) {
        lut.updateImage(img, i, sonar.bins[i], sonar.bin_count);
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_scaled;
    gray.convertTo(gray_scaled, CV_8UC1, 255);

    auto expected_img = cv::imread(PROJECT_DIR + std::string("data/fan1.png"),
        cv::ImreadModes::IMREAD_GRAYSCALE);

    ASSERT_EQ(cv::Scalar(0), cv::sum(expected_img != gray_scaled));
}

TEST_F(SonarToImageLUTTest,
    it_renders_the_sonar_image_when_the_beam_width_is_greater_than_the_step_angle)
{
    sonar.bin_duration = base::Time::fromSeconds(1);
    sonar.speed_of_sound = 1;
    sonar.bin_count = 10;
    sonar.beam_count = 512;
    sonar.setRegularBeamBearings(base::Angle::fromDeg(-65),
        base::Angle::fromDeg(0.25390625));
    sonar.beam_width = base::Angle::fromDeg(0.3);
    size_t bins_size = sonar.beam_count * sonar.bin_count;
    sonar.bins.resize(bins_size, 1);

    SonarToImageLUT lut = SonarToImageLUT(sonar, window_size);

    size_t window_height = lut.getWindowHeight();
    size_t window_width = lut.getWindowWidth();

    cv::Mat img = cv::Mat::zeros(window_height, window_width, CV_8UC3);

    for (size_t i = 0; i < sonar.bins.size(); i++) {
        lut.updateImage(img, i, sonar.bins[i], sonar.bin_count);
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_scaled;
    gray.convertTo(gray_scaled, CV_8UC1, 255);

    auto expected_img = cv::imread(PROJECT_DIR + std::string("data/fan3.png"),
        cv::ImreadModes::IMREAD_GRAYSCALE);

    ASSERT_EQ(cv::Scalar(0), cv::sum(expected_img != gray_scaled));
}

TEST_F(SonarToImageLUTTest, it_renders_the_sonar_image_when_range_greater_then_chord)
{
    sonar.bin_duration = base::Time::fromSeconds(1);
    sonar.speed_of_sound = 1;
    sonar.bin_count = 100;
    sonar.beam_count = 3;
    sonar.setRegularBeamBearings(base::Angle::fromDeg(-10), base::Angle::fromDeg(10));
    sonar.beam_width = base::Angle::fromDeg(10);
    size_t bins_size = sonar.beam_count * sonar.bin_count;
    sonar.bins.resize(bins_size, 1);

    SonarToImageLUT lut = SonarToImageLUT(sonar, window_size);

    size_t window_height = lut.getWindowHeight();
    size_t window_width = lut.getWindowWidth();

    cv::Mat img = cv::Mat::zeros(window_height, window_width, CV_8UC3);

    for (size_t i = 0; i < sonar.bins.size(); i++) {
        lut.updateImage(img, i, sonar.bins[i], sonar.bin_count);
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::Mat gray_scaled;
    gray.convertTo(gray_scaled, CV_8UC1, 255);

    auto expected_img = cv::imread(PROJECT_DIR + std::string("data/fan4.png"),
        cv::ImreadModes::IMREAD_GRAYSCALE);

    ASSERT_EQ(cv::Scalar(0), cv::sum(expected_img != gray_scaled));
}
