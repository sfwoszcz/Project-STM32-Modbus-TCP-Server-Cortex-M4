CC ?= cc
AR ?= ar
PYTHON ?= python3
BUILD ?= build

CPPFLAGS ?= -IApp/include -ITests/stm32
CPPFLAGS += -DMBRTU_ENABLE_DIAGNOSTICS=1
LEGACY_CPPFLAGS := $(filter-out -DMBRTU_ENABLE_DIAGNOSTICS=1,$(CPPFLAGS))
CFLAGS ?= -O2 -g -std=c11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror
LDFLAGS ?=

CORE_SOURCES := \
  App/src/modbus.c \
  App/src/modbus_protocol.c \
  App/src/modbus_crc16.c \
  App/src/modbus_rtu.c \
  App/src/modbus_rtu_diagnostics.c \
  App/src/modbus_rtu_master.c \
  App/src/modbus_rtu_master_transaction.c
CORE_OBJECTS := $(CORE_SOURCES:%.c=$(BUILD)/%.o)
LIBRARY := $(BUILD)/libmodbus_tcp_core.a
CRC_TEST := $(BUILD)/modbus_crc16_tests
PDU_TEST := $(BUILD)/modbus_pdu_tests
RTU_TEST := $(BUILD)/modbus_rtu_tests
RTU_LEGACY_LINK_TEST := $(BUILD)/modbus_rtu_legacy_link_tests
RTU_MASTER_TEST := $(BUILD)/modbus_rtu_master_tests
RTU_MASTER_TRANSACTION_TEST := $(BUILD)/modbus_rtu_master_transaction_tests
RTU_DIAGNOSTICS_TEST := $(BUILD)/modbus_rtu_diagnostics_tests
FC23_TEST := $(BUILD)/modbus_fc23_tests
FC43_DEVICE_ID_TEST := $(BUILD)/modbus_fc43_device_id_tests
RTU_TIMING_TEST := $(BUILD)/modbus_rtu_timing_tests
PROTOCOL_TEST := $(BUILD)/modbus_protocol_tests
SELFTEST := $(BUILD)/register_selftest
POSIX_SERVER := $(BUILD)/modbus_posix_server
LWIP_CHECK_OBJECTS := \
  $(BUILD)/compile-check/modbus_tcp.o \
  $(BUILD)/compile-check/platform_stm32.o

.PHONY: all clean library demo compile-check unit-tests test integration-test ci help stm32-help

all: library demo compile-check unit-tests

help:
	@printf '%s\n' \
	  'make              Build the portable library, tests, demo server, and lwIP compile checks' \
	  'make test         Run CRC, PDU, RTU slave/master/diagnostics/FC23/FC43 tests, TCP, register, and socket tests' \
	  'make ci           Clean and run the complete verification suite' \
	  'make stm32-help   Show CubeMX integration instructions' \
	  'make clean        Remove generated build files'

library: $(LIBRARY)

demo: $(POSIX_SERVER)

compile-check: $(LWIP_CHECK_OBJECTS)

unit-tests: $(CRC_TEST) $(PDU_TEST) $(RTU_TEST) $(RTU_LEGACY_LINK_TEST) $(RTU_MASTER_TEST) \
	$(RTU_MASTER_TRANSACTION_TEST) $(RTU_DIAGNOSTICS_TEST) $(FC23_TEST) \
	$(FC43_DEVICE_ID_TEST) $(RTU_TIMING_TEST) $(PROTOCOL_TEST) $(SELFTEST)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(LIBRARY): $(CORE_OBJECTS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

$(CRC_TEST): Tests/host/test_modbus_crc16.c App/src/modbus_crc16.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(PDU_TEST): Tests/host/test_modbus_pdu.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(RTU_TEST): Tests/host/test_modbus_rtu.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(RTU_LEGACY_LINK_TEST): Tests/host/test_modbus_rtu.c \
	App/src/modbus.c App/src/modbus_protocol.c App/src/modbus_crc16.c \
	App/src/modbus_rtu.c
	@mkdir -p $(dir $@)
	$(CC) $(LEGACY_CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(RTU_MASTER_TEST): Tests/host/test_modbus_rtu_master.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(RTU_MASTER_TRANSACTION_TEST): Tests/host/test_modbus_rtu_master_transaction.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(RTU_DIAGNOSTICS_TEST): Tests/host/test_modbus_rtu_diagnostics.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(FC23_TEST): Tests/host/test_modbus_fc23.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(FC43_DEVICE_ID_TEST): Tests/host/test_modbus_fc43_device_id.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(RTU_TIMING_TEST): Tests/host/test_modbus_rtu_timing.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(PROTOCOL_TEST): Tests/host/test_modbus_protocol.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(SELFTEST): Tests/host/register_selftest_runner.c Tests/stm32/register_selftest.c App/src/modbus.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(POSIX_SERVER): Examples/posix_server.c $(CORE_SOURCES)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ $(LDFLAGS) -o $@

$(BUILD)/compile-check/modbus_tcp.o: App/src/modbus_tcp.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) -ITests/mocks $(CFLAGS) -c $< -o $@

$(BUILD)/compile-check/platform_stm32.o: App/src/platform_stm32.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) -ITests/mocks $(CFLAGS) -c $< -o $@

test: all
	$(CRC_TEST)
	$(PDU_TEST)
	$(RTU_TEST)
	@$(RTU_LEGACY_LINK_TEST) >/dev/null
	@printf '%s\n' 'modbus RTU legacy source-list test: PASS'
	$(RTU_MASTER_TEST)
	$(RTU_MASTER_TRANSACTION_TEST)
	$(RTU_DIAGNOSTICS_TEST)
	$(FC23_TEST)
	$(FC43_DEVICE_ID_TEST)
	$(RTU_TIMING_TEST)
	$(PROTOCOL_TEST)
	$(SELFTEST)
	$(MAKE) integration-test

integration-test: $(POSIX_SERVER)
	@set -eu; \
	  port="$${MODBUS_TEST_PORT:-15020}"; \
	  log="$(BUILD)/posix_server.log"; \
	  $(POSIX_SERVER) "$$port" >"$$log" 2>&1 & pid=$$!; \
	  trap 'kill $$pid 2>/dev/null || true; wait $$pid 2>/dev/null || true' EXIT INT TERM; \
	  $(PYTHON) Tests/host/ci_modbus_smoke.py 127.0.0.1 "$$port"; \
	  kill $$pid 2>/dev/null || true; \
	  wait $$pid 2>/dev/null || true; \
	  trap - EXIT INT TERM

ci: clean test

stm32-help:
	@printf '%s\n' \
	  'This repository supplies the application layer, not board-generated HAL/PHY files.' \
	  'Generate ETH + lwIP (RAW API) with STM32CubeMX, then add:' \
	  '  App/src/modbus.c App/src/modbus_protocol.c App/src/modbus_tcp.c App/src/platform_stm32.c' \
	  'For the RTU ADU and bare-metal byte/timing layer, also add:' \
	  '  App/src/modbus_crc16.c App/src/modbus_rtu.c' \
  'To enable FC07/FC08/FC0B/FC0C diagnostics, also add:' \
  '  App/src/modbus_rtu_diagnostics.c' \
  'and define:' \
  '  MBRTU_ENABLE_DIAGNOSTICS=1' \
	  'and include path:' \
	  '  App/include' \
	  'See README.md and Examples/stm32_cube_main.c for the exact calls.'

clean:
	rm -rf $(BUILD) build-cmake
