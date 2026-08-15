cmake_minimum_required(VERSION 3.22)
project(closed_loop_repository_smoke LANGUAGES CXX)

include(CTest)

add_executable(repository_cpp_smoke tests/repository_cpp_smoke.cpp)
target_compile_features(repository_cpp_smoke PRIVATE cxx_std_17)
target_compile_options(repository_cpp_smoke PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME repository_cpp_smoke COMMAND repository_cpp_smoke)
