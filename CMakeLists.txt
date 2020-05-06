# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Create symbolic links for custom files for lvgl component
execute_process(COMMAND ln -sf ${CMAKE_SOURCE_DIR}/lvgl_CMakeLists.txt ${CMAKE_SOURCE_DIR}/components/lvgl/CMakeLists.txt)
execute_process(COMMAND ln -sf ${CMAKE_SOURCE_DIR}/lv_conf.h ${CMAKE_SOURCE_DIR}/components/lvgl/lv_conf.h)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(m5stack_reflow)
