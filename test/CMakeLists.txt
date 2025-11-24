# This is the project CMakeLists.txt file for the test subproject
cmake_minimum_required(VERSION 3.16)

# Make the main project's components (including packet and its tests) visible.
set(EXTRA_COMPONENT_DIRS "../components")

# List of components that should have their tests included in this test app.
set(TEST_COMPONENTS "packet;led;mesh;system_support" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Build only the components needed for tests, plus their dependencies.
idf_build_set_property(MINIMAL_BUILD ON)
project(packet_component_tests)
