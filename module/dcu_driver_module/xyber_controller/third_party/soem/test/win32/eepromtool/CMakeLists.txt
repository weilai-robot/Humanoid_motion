
set(SOURCES eepromtool.c)
add_executable(eepromtool_win ${SOURCES})
target_link_libraries(eepromtool_win soem)
