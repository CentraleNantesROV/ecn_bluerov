cmake_minimum_required(VERSION 2.8.3)
project(ecn_bluerov)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  ADC.msg
)


catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs std_msgs
#  DEPENDS system_lib
)

# only compile this on ROV

    ## Specify additional locations of header files
    ## Your package locations should be listed before other locations
    include_directories(
    include
    Navio2/C++/Navio/Navio2
    Navio2/C++/Navio/Common
    Navio2/C++/Navio
    ${catkin_INCLUDE_DIRS}
    )


    file(GLOB navio2_SRC
        "Navio2/C++/Navio/Navio2/*.h"
        "Navio2/C++/Navio/Common/*.h"
        "Navio2/C++/Navio/Navio2/*.cpp"
        "Navio2/C++/Navio/Common/*.cpp"
    )
    add_library(navio2 ${navio2_SRC})

    add_executable(pirov_pwm boot/pwm.cpp)
    target_link_libraries(pirov_pwm navio2 ${catkin_LIBRARIES})

    add_executable(pirov_imu  boot/AHRS.cpp boot/AHRS.hpp)
    target_link_libraries(pirov_imu navio2 ${catkin_LIBRARIES})

#    add_executable(pirov_leak boot/leak.cpp)
#    target_link_libraries(pirov_leak navio2 ${catkin_LIBRARIES})

    add_executable(pirov_adc boot/ADC.cpp)
    target_link_libraries(pirov_adc navio2 ${catkin_LIBRARIES})
    add_dependencies(pirov_adc ${${PROJECT_NAME}_EXPORTED_TARGETS})

    #add_executable(pirov_barometer boot/barometer.cpp)
    #target_link_libraries(pirov_barometer navio2 ${catkin_LIBRARIES})
