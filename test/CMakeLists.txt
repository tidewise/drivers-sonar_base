rock_gtest(test_suite suite.cpp
   test_SonarToImageLUT.cpp
   DEPS sonar_base)

target_compile_definitions(test_suite PRIVATE PROJECT_DIR="${PROJECT_SOURCE_DIR}/test/")