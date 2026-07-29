cmake_minimum_required(VERSION 3.16)
project(stm32_modbus_tcp_server VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror)
endif()

add_library(modbus_tcp_core STATIC
    App/src/modbus.c
    App/src/modbus_protocol.c
    App/src/modbus_crc16.c
    App/src/modbus_rtu.c
    App/src/modbus_rtu_diagnostics.c
    App/src/modbus_rtu_server_id.c
    App/src/modbus_rtu_master.c
    App/src/modbus_rtu_master_transaction.c
)
target_include_directories(modbus_tcp_core PUBLIC App/include)
target_compile_definitions(modbus_tcp_core PUBLIC
    MBRTU_ENABLE_DIAGNOSTICS=1
    MBRTU_ENABLE_SERVER_ID=1
)

add_executable(modbus_posix_server Examples/posix_server.c)
target_link_libraries(modbus_posix_server PRIVATE modbus_tcp_core)

add_executable(modbus_crc16_tests Tests/host/test_modbus_crc16.c)
target_link_libraries(modbus_crc16_tests PRIVATE modbus_tcp_core)

add_executable(modbus_pdu_tests Tests/host/test_modbus_pdu.c)
target_link_libraries(modbus_pdu_tests PRIVATE modbus_tcp_core)

add_executable(modbus_rtu_tests Tests/host/test_modbus_rtu.c)
target_link_libraries(modbus_rtu_tests PRIVATE modbus_tcp_core)

add_executable(modbus_rtu_legacy_link_tests
    Tests/host/test_modbus_rtu.c
    App/src/modbus.c
    App/src/modbus_protocol.c
    App/src/modbus_crc16.c
    App/src/modbus_rtu.c
)
target_include_directories(modbus_rtu_legacy_link_tests PRIVATE App/include)

add_executable(modbus_rtu_master_tests Tests/host/test_modbus_rtu_master.c)
target_link_libraries(modbus_rtu_master_tests PRIVATE modbus_tcp_core)

add_executable(modbus_rtu_master_transaction_tests Tests/host/test_modbus_rtu_master_transaction.c)
target_link_libraries(modbus_rtu_master_transaction_tests PRIVATE modbus_tcp_core)

add_executable(modbus_rtu_diagnostics_tests Tests/host/test_modbus_rtu_diagnostics.c)
target_link_libraries(modbus_rtu_diagnostics_tests PRIVATE modbus_tcp_core)

add_executable(modbus_fc11_tests Tests/host/test_modbus_fc11.c)
target_link_libraries(modbus_fc11_tests PRIVATE modbus_tcp_core)

add_executable(modbus_fc23_tests Tests/host/test_modbus_fc23.c)
target_link_libraries(modbus_fc23_tests PRIVATE modbus_tcp_core)

add_executable(modbus_fc43_device_id_tests Tests/host/test_modbus_fc43_device_id.c)
target_link_libraries(modbus_fc43_device_id_tests PRIVATE modbus_tcp_core)

add_executable(modbus_fc20_tests Tests/host/test_modbus_fc20.c)
target_link_libraries(modbus_fc20_tests PRIVATE modbus_tcp_core)

add_executable(modbus_fc21_tests Tests/host/test_modbus_fc21.c)
target_link_libraries(modbus_fc21_tests PRIVATE modbus_tcp_core)

add_executable(modbus_fc22_tests Tests/host/test_modbus_fc22.c)
target_link_libraries(modbus_fc22_tests PRIVATE modbus_tcp_core)

add_executable(modbus_rtu_timing_tests Tests/host/test_modbus_rtu_timing.c)
target_link_libraries(modbus_rtu_timing_tests PRIVATE modbus_tcp_core)

add_executable(modbus_protocol_tests Tests/host/test_modbus_protocol.c)
target_link_libraries(modbus_protocol_tests PRIVATE modbus_tcp_core)

add_executable(register_selftest
    Tests/host/register_selftest_runner.c
    Tests/stm32/register_selftest.c
)
target_include_directories(register_selftest PRIVATE Tests/stm32)
target_link_libraries(register_selftest PRIVATE modbus_tcp_core)

add_library(lwip_transport_compile_check OBJECT
    App/src/modbus_tcp.c
    App/src/platform_stm32.c
)
target_include_directories(lwip_transport_compile_check PRIVATE
    App/include
    Tests/mocks
)

enable_testing()
add_test(NAME crc16 COMMAND modbus_crc16_tests)
add_test(NAME pdu COMMAND modbus_pdu_tests)
add_test(NAME rtu COMMAND modbus_rtu_tests)
add_test(NAME rtu_legacy_link COMMAND modbus_rtu_legacy_link_tests)
add_test(NAME rtu_master COMMAND modbus_rtu_master_tests)
add_test(NAME rtu_master_transaction COMMAND modbus_rtu_master_transaction_tests)
add_test(NAME rtu_diagnostics COMMAND modbus_rtu_diagnostics_tests)
add_test(NAME fc11 COMMAND modbus_fc11_tests)
add_test(NAME fc23 COMMAND modbus_fc23_tests)
add_test(NAME fc43_device_id COMMAND modbus_fc43_device_id_tests)
add_test(NAME fc20 COMMAND modbus_fc20_tests)
add_test(NAME fc21 COMMAND modbus_fc21_tests)
add_test(NAME fc22 COMMAND modbus_fc22_tests)
add_test(NAME rtu_timing COMMAND modbus_rtu_timing_tests)
add_test(NAME protocol COMMAND modbus_protocol_tests)
add_test(NAME register_map COMMAND register_selftest)
