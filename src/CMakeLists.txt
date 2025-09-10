rock_library(sonar_base
    SOURCES SonarToImageLUT.cpp
    HEADERS SonarToImageLUT.hpp
    DEPS_PKGCONFIG ${OPENCV_PACKAGE} base-types)
