cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_STANDARD 11)

idf_component_register(
    SRCS
        esp32-rmt-uart-v5.c
    INCLUDE_DIRS
        .
    REQUIRES
        rmt_uart
        yys
        ze08
)
