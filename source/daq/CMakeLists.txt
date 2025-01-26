cmake_minimum_required(VERSION 3.13)

# Setup Component name based on directory
get_filename_component(COMPONENT_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(TOLOWER ${COMPONENT_NAME} COMPONENT_NAME)
set(TARGET_NAME ${COMPONENT_NAME}.elf)
add_executable(${TARGET_NAME})
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_SOURCE_DIR})

# Propreties are set in order to make the common component
set_target_properties(${TARGET_NAME} PROPERTIES
    COMPONENT_NAME ${COMPONENT_NAME}
    COMPONENT_DIR  ${CMAKE_CURRENT_LIST_DIR}
    LINKER_SCRIPT  "STM32F407VGTx_FLASH"
    COMMON_LIBS    "CMSIS_F407;PHAL_F407;QUEUE;CAN_PARSE_F407;FREERTOS"
)
COMMON_FIRMWARE_COMPONENT(${TARGET_NAME})
