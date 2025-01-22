set(TARGET_NAME EEPROM_SPI)
add_library(${TARGET_NAME})

# Find all .c sources in project
target_sources(${TARGET_NAME} PRIVATE "eeprom_spi.c")

# Find directories for '#include'
# For libraries, these directories are all referenced to the top level firmware directory, CMAKE_SOURCE_DIR
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_SOURCE_DIR})

target_link_libraries(${TARGET_NAME} "PHAL_F407")