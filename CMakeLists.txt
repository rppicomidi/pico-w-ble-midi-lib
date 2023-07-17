cmake_minimum_required(VERSION 3.13)

set(target_lib ble_midi_service_lib)
add_library(${target_lib} INTERFACE)
target_sources(${target_lib} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/midi_service_server.c
    ${CMAKE_CURRENT_LIST_DIR}/midi_service_stream_handler.c
)

target_include_directories(${target_lib} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../ring_buffer_lib ring_buffer_lib
)

target_include_directories(${target_lib} INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(${target_lib} INTERFACE
    pico_stdlib
    pico_btstack_ble
    ring_buffer_lib
)