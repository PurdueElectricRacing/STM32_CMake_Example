# CMAKE file for building STM32CubeL4 CMSIS module

set(LIB_PATH "${CMAKE_SOURCE_DIR}/common/freertos_M4F/FreeRTOS-Kernel")

function(make_freertos_library LIB_NAME CMSIS_LIB)
    add_library(${LIB_NAME} STATIC)

    target_include_directories(${LIB_NAME} 
        PUBLIC ${LIB_PATH}/include
        PUBLIC ${LIB_PATH}/portable/GCC/ARM_CM4F
        PUBLIC $<TARGET_PROPERTY:${CMSIS_LIB},INTERFACE_INCLUDE_DIRECTORIES>  # Inherit CMSIS include directories
    )

    file(GLOB glob_sources "${LIB_PATH}/*.c" "${LIB_PATH}/portable/MemMang/*.c" "${LIB_PATH}/portable/GCC/ARM_CM4F/*.c")
    target_sources(${LIB_NAME} 
        PUBLIC ${glob_sources}
    )

    target_link_libraries(${LIB_NAME} PUBLIC ${CMSIS_LIB})
    message(STATUS "FreeRTOS include directories: ${LIB_PATH}/include;${LIB_PATH}/portable/GCC/ARM_CM4F")

endfunction()

# Create multiple libraries with different defines
make_freertos_library(FREERTOS_LIB CMSIS_F407)
