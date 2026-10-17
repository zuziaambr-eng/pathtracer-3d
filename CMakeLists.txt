cmake_minimum_required(VERSION 3.14)
project(PathTracer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Wersja konsolowa CPU
add_executable(pathtracer main.cpp)

# Wersja z interfejsem graficznym (GUI)
add_executable(pathtracer_gui WIN32 gui_main.cpp)
target_link_libraries(pathtracer_gui PRIVATE gdi32 user32 comctl32)

# Opcjonalna wersja CUDA (jeśli włączona)
if(DEFINED CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    add_executable(pathtracer_cuda main.cu)
    set_property(TARGET pathtracer_cuda PROPERTY CUDA_ARCHITECTURES 86 native)
endif()

if(MSVC)
    target_compile_options(pathtracer PRIVATE /O2 /W4)
    target_compile_options(pathtracer_gui PRIVATE /O2 /W4)
else()
    target_compile_options(pathtracer PRIVATE -O3 -Wall)
    target_compile_options(pathtracer_gui PRIVATE -O3 -Wall)
endif()
