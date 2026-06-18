
set(SOURCES slaveinfo.c)
add_executable(slaveinfo_win ${SOURCES})
target_link_libraries(slaveinfo_win soem)
install(TARGETS slaveinfo_win DESTINATION bin)
