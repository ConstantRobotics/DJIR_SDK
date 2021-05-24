cmake_minimum_required(VERSION 3.13)



###############################################################################
## PROJECT
## name and version
###############################################################################
project(DJIR_Examples LANGUAGES C CXX)



###############################################################################
## CONFIGURATION
## project configuration
###############################################################################
# Tests
SET(${PARENT}_DJIR_CONSOLE_TEST                        ON CACHE BOOL  "" FORCE)



###############################################################################
## INCLUDING SUBDIRECTORIES
## Adding subdirectories according to the configuration
###############################################################################
# Tests
if (${PARENT}_DJIR_CONSOLE_TEST)
    add_subdirectory(ConsoleTest)
endif()
