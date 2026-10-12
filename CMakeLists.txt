cmake_minimum_required(VERSION 3.22)

# App code lives in src/ (registered as the "src" component instead of IDF's default main/)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)

# Only build src and what it REQUIRES, not every IDF component
set(COMPONENTS src)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(toy-robot)
