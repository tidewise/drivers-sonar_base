rock_library(sonar_base
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(sonar_base_bin Main.cpp
    DEPS sonar_base)

