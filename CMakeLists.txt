cmake_minimum_required(VERSION 3.22.1)
project(SMNBody)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_FLAGS "-O0")

include_directories(include)

add_executable(SMNBody main.cpp
        src/TreeNode.cpp
        include/TreeNode.h
        src/Particle.cpp
        include/Particle.h
        include/types.h
        src/Vector2.cpp
        include/Vector2.h
        src/Sim.cpp
        include/Sim.h
        include/nlohmann/json.hpp
        src/GravityFunction.cpp
        include/GravityFunction.h
)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(SMNBody PUBLIC OpenMP::OpenMP_CXX)
endif()
