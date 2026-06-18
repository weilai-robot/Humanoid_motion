
set(SOURCES simple_test.c)
add_executable(simple_test_win ${SOURCES})
target_link_libraries(simple_test_win soem)
