cmake_minimum_required(VERSION 3.13)



###############################################################################
## PROJECT
## name and version
###############################################################################
project(DJIR_SDK LANGUAGES CXX)



###############################################################################
## INCLUDING SUBDIRECTORIES
## Adding subdirectories according to the configuration
###############################################################################
# 3rdparty
add_subdirectory(3rdparty)
# src
add_subdirectory(src)
# examples
#add_subdirectory(examples)
