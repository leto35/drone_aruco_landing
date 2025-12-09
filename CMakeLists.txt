
cmake_minimum_required(VERSION 3.10)
project(my_drone_clean)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Indique à CMake où trouver MAVSDK
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/MAVSDK")

find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)

# Optional Gazebo support
option(HAVE_GZ "Enable Gazebo transport support" ON)
if(HAVE_GZ)
    find_package(gz-transport13 QUIET)
    find_package(gz-msgs10 QUIET)

    if(gz-transport13_FOUND AND gz-msgs10_FOUND)
        message(STATUS "Gazebo transport support enabled")
        add_definitions(-DHAVE_GZ)
        set(GZ_LIBS gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
    else()
        message(WARNING "Gazebo libs not found, disabling gz-topic support")
        set(HAVE_GZ OFF)
        set(GZ_LIBS "")
    endif()
else()
    message(STATUS "Gazebo transport support disabled")
    set(GZ_LIBS "")
endif()

# Programme principal - Version propre et simple
# add_executable(my_drone_clean my_drone_clean.cpp)
# target_link_libraries(my_drone_clean MAVSDK::mavsdk ${OpenCV_LIBS} ${GZ_LIBS})
# target_include_directories(my_drone_clean PRIVATE /usr/local/include/mavsdk ${OpenCV_INCLUDE_DIRS})

# Programme avec contrôleur PID pour cible mobile
add_executable(my_drone_pid my_drone_pid.cpp)
target_link_libraries(my_drone_pid MAVSDK::mavsdk ${OpenCV_LIBS} ${GZ_LIBS})
target_include_directories(my_drone_pid PRIVATE /usr/local/include/mavsdk ${OpenCV_INCLUDE_DIRS})

# Programme avec contrôleur proportionnel simple pour cible fixe (Phase 1)
add_executable(my_drone_fixed_target my_drone_fixed_target.cpp)
target_link_libraries(my_drone_fixed_target MAVSDK::mavsdk ${OpenCV_LIBS} ${GZ_LIBS})
target_include_directories(my_drone_fixed_target PRIVATE /usr/local/include/mavsdk ${OpenCV_INCLUDE_DIRS})
