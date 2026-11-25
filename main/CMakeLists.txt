cmake_minimum_required(VERSION 3.16)

if(DEFINED APP_NAME)
    set(PROJ_NAME ${APP_NAME})
elseif(DEFINED ENV{APP_NAME})
    set(PROJ_NAME $ENV{APP_NAME})
else()
    set(PROJ_NAME "esp32c6_arm_receiver")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(${PROJ_NAME})