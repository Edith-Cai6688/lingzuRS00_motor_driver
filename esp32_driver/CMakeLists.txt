# 以下样板行必须按此确切顺序出现在项目的 CMakeLists 中
# 以便 cmake 能够正常工作
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(twai_self_test_example)
