#include "modbus.h"
#include "modbus_crc16.h"
#include "modbus_protocol.h"
#include "modbus_rtu.h"
#include "modbus_rtu_diagnostics.h"
#include "modbus_rtu_master.h"
#include "modbus_rtu_master_transaction.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

static size_t append_crc(uint8_t *adu, size_t length_without_crc)
{
    uint16_t crc = mb_crc16(adu, length_without_crc);

    adu[length_without_crc] = (uint8_t)crc;
    adu[length_without_crc + 1u] = (uint8_t)(crc >> 8u);
    return length_without_crc + 2u;
}

static size_t make_rtu_request(uint8_t address,
                               const uint8_t *pdu,
                               size_t pdu_length,
                               uint8_t *adu)
{
    adu[0] = address;
    memcpy(&adu[1], pdu, pdu_length);
    return append_crc(adu, 1u + pdu_length);
}

static int policy_allow_all(void *context,
                            mbrtu_diagnostics_action_t action,
                            uint16_t request_data)
{
    unsigned *calls = context;

    (void)action;
    (void)request_data;
    if (calls != NULL) {
        ++(*calls);
    }
    return 1;
}

static int policy_deny_all(void *context,
                           mbrtu_diagnostics_action_t action,
                           uint16_t request_data)
{
    (void)context;
    (void)action;
    (void)request_data;
    return 0;
}

static int process_diag(mbrtu_diagnostics_t *diagnostics,
                        mbrtu_diagnostics_policy_fn policy,
                        void *policy_context,
                        uint8_t address,
                        const uint8_t *pdu,
                        size_t pdu_length,
                        uint8_t *response,
                        size_t *response_length)
{
    uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = make_rtu_request(address,
                                             pdu,
                                             pdu_length,
                                             request);

    return mbrtu_process_adu_with_diagnostics(1u,
                                               request,
                                               request_length,
                                               response,
                                               MODBUS_RTU_ADU_MAX_SIZE,
                                               response_length,
                                               diagnostics,
                                               policy,
                                               policy_context);
}


typedef struct {
    mbrtu_context_t context;
    mbrtu_diagnostics_t diagnostics;
    uint8_t receive_a[8];
    uint8_t receive_b[8];
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t transmitted[MODBUS_RTU_ADU_MAX_SIZE];
    size_t transmitted_length;
    unsigned transmit_count;
} diagnostics_timing_fixture_t;

static int capture_diagnostics_transmit(void *context,
                                        const uint8_t *adu,
                                        size_t adu_length)
{
    diagnostics_timing_fixture_t *fixture = context;

    if (fixture == NULL || adu == NULL ||
        adu_length > sizeof(fixture->transmitted)) {
        return -1;
    }
    memcpy(fixture->transmitted, adu, adu_length);
    fixture->transmitted_length = adu_length;
    ++fixture->transmit_count;
    return 0;
}

static void advance_diagnostics_ticks(mbrtu_context_t *context,
                                      uint32_t tick_count)
{
    for (uint32_t tick = 0u; tick < tick_count; ++tick) {
        mbrtu_on_50us_tick_isr(context);
    }
}

static void receive_diagnostics_bytes(mbrtu_context_t *context,
                                      const uint8_t *adu,
                                      size_t adu_length)
{
    for (size_t index = 0u; index < adu_length; ++index) {
        mbrtu_on_rx_byte_isr(context, adu[index]);
    }
}

static int test_timing_layer_diagnostics_integration(void)
{
    diagnostics_timing_fixture_t fixture;
    mbrtu_config_t config;
    uint8_t request[16];
    uint8_t event = 0u;
    size_t request_length;
    const uint8_t fc07[] = {MBRTU_FC_READ_EXCEPTION_STATUS};
    const uint8_t overflowing_local[] = {
        1u, 3u, 0u, 0u, 0u, 1u, 0u, 0u, 0u
    };
    const uint8_t overflowing_other[] = {
        2u, 3u, 0u, 0u, 0u, 1u, 0u, 0u, 0u
    };

    memset(&fixture, 0, sizeof(fixture));
    mbrtu_diagnostics_init(&fixture.diagnostics);
    mbrtu_diagnostics_set_exception_status(&fixture.diagnostics, 0x5Au);

    config.slave_address = 1u;
    config.baud_rate = 115200u;
    config.data_bits = 8u;
    config.parity_bits = 1u;
    config.stop_bits = 1u;
    config.receive_buffer_a = fixture.receive_a;
    config.receive_buffer_b = fixture.receive_b;
    config.receive_buffer_capacity = sizeof(fixture.receive_a);
    config.response_buffer = fixture.response;
    config.response_buffer_capacity = sizeof(fixture.response);
    config.transmit = capture_diagnostics_transmit;
    config.user_context = &fixture;

    CHECK(mbrtu_init_with_diagnostics(&fixture.context,
                                      &config,
                                      &fixture.diagnostics,
                                      policy_allow_all,
                                      NULL) == 0);

    request_length = make_rtu_request(1u, fc07, sizeof(fc07), request);
    receive_diagnostics_bytes(&fixture.context, request, request_length);
    advance_diagnostics_ticks(
        &fixture.context,
        fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_RESPONSE_SENT);
    CHECK(fixture.transmit_count == 1u);
    CHECK(fixture.transmitted_length == 5u);
    CHECK(fixture.transmitted[1] == MBRTU_FC_READ_EXCEPTION_STATUS);
    CHECK(fixture.transmitted[2] == 0x5Au);
    CHECK(mb_crc16(fixture.transmitted, fixture.transmitted_length) == 0u);
    CHECK(fixture.diagnostics.message_count == 1u);
    CHECK(fixture.diagnostics.server_message_count == 1u);
    CHECK(fixture.diagnostics.event_counter == 1u);

    receive_diagnostics_bytes(&fixture.context,
                              overflowing_local,
                              sizeof(overflowing_local));
    advance_diagnostics_ticks(
        &fixture.context,
        fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_FRAME_DROPPED);
    CHECK(fixture.transmit_count == 1u);
    CHECK(fixture.diagnostics.message_count == 2u);
    CHECK(fixture.diagnostics.bus_character_overrun_count == 1u);
    CHECK(mbrtu_diagnostics_get_event(&fixture.diagnostics, 0u, &event) == 0);
    CHECK((event & 0x10u) != 0u);

    receive_diagnostics_bytes(&fixture.context,
                              overflowing_other,
                              sizeof(overflowing_other));
    advance_diagnostics_ticks(
        &fixture.context,
        fixture.context.timing.byte_complete_t3_5_ticks);
    CHECK(mbrtu_poll(&fixture.context) == MBRTU_POLL_FRAME_DROPPED);
    CHECK(fixture.diagnostics.message_count == 3u);
    CHECK(fixture.diagnostics.bus_character_overrun_count == 1u);
    return EXIT_SUCCESS;
}

static int test_slave_read_functions_and_lengths(void)
{
    mbrtu_diagnostics_t diagnostics;
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    size_t response_length = 0u;
    const uint8_t fc07[] = {MBRTU_FC_READ_EXCEPTION_STATUS};
    const uint8_t fc07_bad[] = {MBRTU_FC_READ_EXCEPTION_STATUS, 0x00u};
    const uint8_t fc0b[] = {MBRTU_FC_GET_COMM_EVENT_COUNTER};
    const uint8_t fc0c[] = {MBRTU_FC_GET_COMM_EVENT_LOG};

    mbrtu_diagnostics_init(&diagnostics);
    mbrtu_diagnostics_set_exception_status(&diagnostics, 0xA5u);
    mbrtu_diagnostics_set_communication_status(&diagnostics, 0xFFFFu);
    diagnostics.event_counter = 0x1234u;

    {
        uint8_t request[MODBUS_RTU_ADU_MAX_SIZE];
        size_t request_length = make_rtu_request(1u,
                                                 fc07,
                                                 sizeof(fc07),
                                                 request);

        CHECK(mbrtu_process_adu(1u,
                                request,
                                request_length,
                                response,
                                sizeof(response),
                                &response_length) == MBRTU_RESPONSE_READY);
        CHECK(response[1] ==
              (uint8_t)(MBRTU_FC_READ_EXCEPTION_STATUS | 0x80u));
        CHECK(response[2] == MBRTUM_EXCEPTION_ILLEGAL_FUNCTION);
    }

    CHECK(process_diag(&diagnostics, NULL, NULL, 1u,
                       fc07, sizeof(fc07), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response_length == 5u);
    CHECK(response[1] == MBRTU_FC_READ_EXCEPTION_STATUS);
    CHECK(response[2] == 0xA5u);
    CHECK(mb_crc16(response, response_length) == 0u);

    CHECK(process_diag(&diagnostics, NULL, NULL, 1u,
                       fc07_bad, sizeof(fc07_bad), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response[1] == (uint8_t)(MBRTU_FC_READ_EXCEPTION_STATUS | 0x80u));
    CHECK(response[2] == MBRTUM_EXCEPTION_ILLEGAL_DATA_VALUE);

    diagnostics.event_counter = 0x1234u;
    CHECK(process_diag(&diagnostics, NULL, NULL, 1u,
                       fc0b, sizeof(fc0b), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response_length == 8u);
    CHECK(response[1] == MBRTU_FC_GET_COMM_EVENT_COUNTER);
    CHECK(response[2] == 0xFFu && response[3] == 0xFFu);
    CHECK(response[4] == 0x12u && response[5] == 0x34u);

    CHECK(process_diag(&diagnostics, NULL, NULL, 1u,
                       fc0c, sizeof(fc0c), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response[1] == MBRTU_FC_GET_COMM_EVENT_LOG);
    CHECK(response[2] >= 6u && response[2] <= 70u);
    CHECK(response_length == (size_t)response[2] + 5u);
    CHECK(mb_crc16(response, response_length) == 0u);
    return EXIT_SUCCESS;
}

static int test_fc08_policy_listen_only_and_reset(void)
{
    mbrtu_diagnostics_t diagnostics;
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    size_t response_length = 0u;
    unsigned policy_calls = 0u;
    const uint8_t return_query[] = {
        MBRTU_FC_DIAGNOSTICS, 0x00u, 0x00u, 0xA5u, 0x37u,
        0x12u, 0x34u
    };
    const uint8_t return_register[] = {
        MBRTU_FC_DIAGNOSTICS, 0x00u, 0x02u, 0x00u, 0x00u
    };
    const uint8_t malformed_odd[] = {
        MBRTU_FC_DIAGNOSTICS, 0x00u, 0x00u, 0x01u
    };
    const uint8_t clear_counters[] = {
        MBRTU_FC_DIAGNOSTICS, 0x00u, 0x0Au, 0x00u, 0x00u
    };
    const uint8_t force_listen[] = {
        MBRTU_FC_DIAGNOSTICS, 0x00u, 0x04u, 0x00u, 0x00u
    };
    const uint8_t restart[] = {
        MBRTU_FC_DIAGNOSTICS, 0x00u, 0x01u, 0xFFu, 0x00u
    };
    const uint8_t ordinary_read[] = {0x03u, 0x00u, 0x00u, 0x00u, 0x01u};

    mbrtu_diagnostics_init(&diagnostics);
    mbrtu_diagnostics_set_register(&diagnostics, 0xBEEFu);

    CHECK(process_diag(&diagnostics, NULL, NULL, 1u,
                       return_query, sizeof(return_query), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response_length == 1u + sizeof(return_query) + 2u);
    CHECK(memcmp(&response[1], return_query, sizeof(return_query)) == 0);

    CHECK(process_diag(&diagnostics, NULL, NULL, 1u,
                       return_register, sizeof(return_register), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response[4] == 0xBEu && response[5] == 0xEFu);

    CHECK(process_diag(&diagnostics, NULL, NULL, 1u,
                       malformed_odd, sizeof(malformed_odd), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response[1] == (uint8_t)(MBRTU_FC_DIAGNOSTICS | 0x80u));
    CHECK(response[2] == MBRTUM_EXCEPTION_ILLEGAL_DATA_VALUE);

    diagnostics.event_counter = 33u;
    CHECK(process_diag(&diagnostics, policy_deny_all, NULL, 1u,
                       clear_counters, sizeof(clear_counters), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(response[1] == (uint8_t)(MBRTU_FC_DIAGNOSTICS | 0x80u));
    CHECK(diagnostics.event_counter == 33u);

    diagnostics.diagnostic_register = 0xBEEFu;
    CHECK(mbrtu_diagnostics_process_pdu(
              &diagnostics,
              policy_allow_all,
              &policy_calls,
              1u,
              clear_counters,
              sizeof(clear_counters),
              response,
              4u,
              &response_length) == MBRTU_DIAGNOSTICS_PDU_ERROR);
    CHECK(response_length == 0u);
    CHECK(diagnostics.event_counter == 33u);
    CHECK(diagnostics.diagnostic_register == 0xBEEFu);

    CHECK(process_diag(&diagnostics, policy_allow_all, &policy_calls, 1u,
                       clear_counters, sizeof(clear_counters), response,
                       &response_length) == MBRTU_RESPONSE_READY);
    CHECK(diagnostics.event_counter == 0u);
    CHECK(diagnostics.diagnostic_register == 0u);

    CHECK(process_diag(&diagnostics, policy_allow_all, &policy_calls, 1u,
                       force_listen, sizeof(force_listen), response,
                       &response_length) == MBRTU_NO_RESPONSE);
    CHECK(response_length == 0u);
    CHECK(mbrtu_diagnostics_is_listen_only(&diagnostics));
    CHECK((mbrtu_diagnostics_take_pending_actions(&diagnostics) &
           MBRTU_DIAGNOSTICS_PENDING_ENTER_LISTEN_ONLY) != 0u);

    mb_init();
    CHECK(process_diag(&diagnostics, policy_allow_all, &policy_calls, 1u,
                       ordinary_read, sizeof(ordinary_read), response,
                       &response_length) == MBRTU_NO_RESPONSE);
    CHECK(response_length == 0u);

    CHECK(process_diag(&diagnostics, policy_allow_all, &policy_calls, 1u,
                       restart, sizeof(restart), response,
                       &response_length) == MBRTU_NO_RESPONSE);
    CHECK(!mbrtu_diagnostics_is_listen_only(&diagnostics));
    CHECK((mbrtu_diagnostics_take_pending_actions(&diagnostics) &
           MBRTU_DIAGNOSTICS_PENDING_RESTART) != 0u);
    CHECK(diagnostics.event_log_count == 1u);
    CHECK(policy_calls >= 3u);
    return EXIT_SUCCESS;
}

static int test_broadcast_and_event_log_overflow(void)
{
    mbrtu_diagnostics_t diagnostics;
    uint8_t response[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t event = 0u;
    size_t response_length = 0u;
    const uint8_t read_status[] = {MBRTU_FC_READ_EXCEPTION_STATUS};
    const uint8_t clear_counters[] = {
        MBRTU_FC_DIAGNOSTICS, 0x00u, 0x0Au, 0x00u, 0x00u
    };

    mbrtu_diagnostics_init(&diagnostics);
    diagnostics.event_counter = 99u;

    CHECK(process_diag(&diagnostics, policy_allow_all, NULL, 0u,
                       read_status, sizeof(read_status), response,
                       &response_length) == MBRTU_NO_RESPONSE);
    CHECK(diagnostics.event_counter == 99u);
    CHECK(diagnostics.server_no_response_count == 0u);

    CHECK(process_diag(&diagnostics, policy_allow_all, NULL, 0u,
                       clear_counters, sizeof(clear_counters), response,
                       &response_length) == MBRTU_NO_RESPONSE);
    CHECK(diagnostics.event_counter == 0u);
    CHECK(diagnostics.server_no_response_count == 0u);
    CHECK(diagnostics.event_log_count > 0u);

    mbrtu_diagnostics_init(&diagnostics);
    for (unsigned i = 0u; i < 80u; ++i) {
        mbrtu_diagnostics_note_bus_message(&diagnostics,
                                           (uint8_t)(i & 1u));
    }
    CHECK(diagnostics.event_log_count ==
          MBRTU_DIAGNOSTICS_EVENT_LOG_CAPACITY);
    CHECK(mbrtu_diagnostics_get_event(&diagnostics, 0u, &event) == 0);
    CHECK(event == 0xC0u);
    CHECK(mbrtu_diagnostics_get_event(
              &diagnostics,
              MBRTU_DIAGNOSTICS_EVENT_LOG_CAPACITY - 1u,
              &event) == 0);
    CHECK(event == 0x80u);
    CHECK(mbrtu_diagnostics_get_event(
              &diagnostics,
              MBRTU_DIAGNOSTICS_EVENT_LOG_CAPACITY,
              &event) != 0);
    return EXIT_SUCCESS;
}

static int test_fc08_counters_clear_and_saturation(void)
{
    mbrtu_diagnostics_t diagnostics;
    uint8_t request_pdu[5];
    uint8_t response_pdu[8];
    size_t response_pdu_length = 0u;
    const struct {
        uint16_t subfunction;
        uint16_t value;
    } cases[] = {
        {MBRTU_DIAG_SUB_RETURN_BUS_MESSAGE_COUNT, 0x1001u},
        {MBRTU_DIAG_SUB_RETURN_BUS_COMM_ERROR_COUNT, 0x1002u},
        {MBRTU_DIAG_SUB_RETURN_BUS_EXCEPTION_COUNT, 0x1003u},
        {MBRTU_DIAG_SUB_RETURN_SERVER_MESSAGE_COUNT, 0x1004u},
        {MBRTU_DIAG_SUB_RETURN_SERVER_NO_RESPONSE_COUNT, 0x1005u},
        {MBRTU_DIAG_SUB_RETURN_SERVER_NAK_COUNT, 0x1006u},
        {MBRTU_DIAG_SUB_RETURN_SERVER_BUSY_COUNT, 0x1007u},
        {MBRTU_DIAG_SUB_RETURN_BUS_OVERRUN_COUNT, 0x1008u}
    };

    mbrtu_diagnostics_init(&diagnostics);
    diagnostics.message_count = cases[0].value;
    diagnostics.bus_communication_error_count = cases[1].value;
    diagnostics.bus_exception_error_count = cases[2].value;
    diagnostics.server_message_count = cases[3].value;
    diagnostics.server_no_response_count = cases[4].value;
    diagnostics.server_nak_count = cases[5].value;
    diagnostics.server_busy_count = cases[6].value;
    diagnostics.bus_character_overrun_count = cases[7].value;

    request_pdu[0] = MBRTU_FC_DIAGNOSTICS;
    request_pdu[3] = 0u;
    request_pdu[4] = 0u;
    for (size_t i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        request_pdu[1] = (uint8_t)(cases[i].subfunction >> 8u);
        request_pdu[2] = (uint8_t)cases[i].subfunction;
        CHECK(mbrtu_diagnostics_process_pdu(
                  &diagnostics,
                  NULL,
                  NULL,
                  1u,
                  request_pdu,
                  sizeof(request_pdu),
                  response_pdu,
                  sizeof(response_pdu),
                  &response_pdu_length) ==
              MBRTU_DIAGNOSTICS_PDU_RESPONSE);
        CHECK(response_pdu_length == 5u);
        CHECK(response_pdu[1] == request_pdu[1]);
        CHECK(response_pdu[2] == request_pdu[2]);
        CHECK(response_pdu[3] == (uint8_t)(cases[i].value >> 8u));
        CHECK(response_pdu[4] == (uint8_t)cases[i].value);
    }

    request_pdu[1] = 0u;
    request_pdu[2] = (uint8_t)MBRTU_DIAG_SUB_CLEAR_OVERRUN_COUNTER;
    CHECK(mbrtu_diagnostics_process_pdu(
              &diagnostics,
              policy_allow_all,
              NULL,
              1u,
              request_pdu,
              sizeof(request_pdu),
              response_pdu,
              sizeof(response_pdu),
              &response_pdu_length) == MBRTU_DIAGNOSTICS_PDU_RESPONSE);
    CHECK(diagnostics.bus_character_overrun_count == 0u);

    diagnostics.message_count = UINT16_MAX;
    diagnostics.event_counter = UINT16_MAX;
    diagnostics.server_message_count = UINT16_MAX;
    diagnostics.server_no_response_count = UINT16_MAX;
    diagnostics.bus_communication_error_count = UINT16_MAX;
    diagnostics.bus_exception_error_count = UINT16_MAX;
    diagnostics.server_nak_count = UINT16_MAX;
    diagnostics.server_busy_count = UINT16_MAX;
    diagnostics.bus_character_overrun_count = UINT16_MAX;
    mbrtu_diagnostics_note_bus_message(&diagnostics, 0u);
    mbrtu_diagnostics_note_server_message(&diagnostics);
    mbrtu_diagnostics_note_normal_completion(&diagnostics, 1u, 0u);
    mbrtu_diagnostics_note_exception(&diagnostics, 6u);
    mbrtu_diagnostics_note_exception(&diagnostics, 7u);
    mbrtu_diagnostics_note_communication_error(&diagnostics, 0u, 1u);
    CHECK(diagnostics.message_count == UINT16_MAX);
    CHECK(diagnostics.event_counter == UINT16_MAX);
    CHECK(diagnostics.server_message_count == UINT16_MAX);
    CHECK(diagnostics.server_no_response_count == UINT16_MAX);
    CHECK(diagnostics.bus_communication_error_count == UINT16_MAX);
    CHECK(diagnostics.bus_exception_error_count == UINT16_MAX);
    CHECK(diagnostics.server_nak_count == UINT16_MAX);
    CHECK(diagnostics.server_busy_count == UINT16_MAX);
    CHECK(diagnostics.bus_character_overrun_count == UINT16_MAX);
    return EXIT_SUCCESS;
}

static int test_master_builders_and_decoders(void)
{
    mbrtum_request_t request;
    mbrtum_response_t response_view;
    mbrtum_diagnostics_response_t diagnostics_response;
    mbrtum_comm_event_log_t event_log;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[MODBUS_RTU_ADU_MAX_SIZE];
    size_t request_length = 0u;
    size_t response_length;
    uint8_t status8 = 0u;
    uint8_t event = 0u;
    uint16_t subfunction = 0u;
    uint16_t value = 0u;
    uint16_t status = 0u;
    uint16_t count = 0u;
    const uint8_t query_data[] = {0xA5u, 0x37u, 0x12u, 0x34u};
    const uint8_t zero_word[] = {0x00u, 0x00u};
    const uint8_t odd_data[] = {0x00u};

    CHECK(mbrtum_build_diagnostics_request(
              1u,
              0x0003u,
              zero_word,
              sizeof(zero_word),
              &request,
              request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_ERROR_FUNCTION);
    CHECK(mbrtum_build_diagnostics_request(
              1u,
              MBRTU_DIAG_SUB_RETURN_QUERY_DATA,
              odd_data,
              sizeof(odd_data),
              &request,
              request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_ERROR_VALUE);
    CHECK(mbrtum_build_diagnostics_request(
              0u,
              MBRTU_DIAG_SUB_RETURN_DIAGNOSTIC_REGISTER,
              zero_word,
              sizeof(zero_word),
              &request,
              request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_ERROR_SLAVE_ADDRESS);
    CHECK(mbrtum_build_diagnostics_request(
              1u,
              MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY,
              zero_word,
              sizeof(zero_word),
              &request,
              request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    CHECK(request.expects_response == 0u);

    CHECK(mbrtum_build_read_exception_status_request(
              1u, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    CHECK(request_length == 4u);
    response_adu[0] = 1u;
    response_adu[1] = MBRTUM_FC_READ_EXCEPTION_STATUS;
    response_adu[2] = 0x6Du;
    response_length = append_crc(response_adu, 3u);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response_view) == MBRTUM_OK);
    CHECK(mbrtum_get_exception_status(&response_view, &status8) == MBRTUM_OK);
    CHECK(status8 == 0x6Du);

    CHECK(mbrtum_build_diagnostics_request(
              1u,
              MBRTU_DIAG_SUB_RETURN_QUERY_DATA,
              query_data,
              sizeof(query_data),
              &request,
              request_adu,
              sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    memcpy(response_adu, request_adu, request_length);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  request_length,
                                  &response_view) ==
          MBRTUM_ERROR_REQUEST_DATA_REQUIRED);
    CHECK(mbrtum_process_response_with_request_adu(
              &request,
              request_adu,
              request_length,
              response_adu,
              request_length,
              &response_view) == MBRTUM_OK);
    CHECK(mbrtum_get_diagnostics_response(
              &response_view,
              &diagnostics_response) == MBRTUM_OK);
    CHECK(diagnostics_response.subfunction ==
          MBRTU_DIAG_SUB_RETURN_QUERY_DATA);
    CHECK(diagnostics_response.data_length == sizeof(query_data));
    CHECK(memcmp(diagnostics_response.data,
                 query_data,
                 sizeof(query_data)) == 0);
    CHECK(mbrtum_get_diagnostics_word(&response_view,
                                      &subfunction,
                                      &value) == MBRTUM_OK);
    CHECK(subfunction == MBRTU_DIAG_SUB_RETURN_QUERY_DATA);
    CHECK(value == 0xA537u);

    response_adu[4] ^= 0x01u;
    response_length = append_crc(response_adu, request_length - 2u);
    CHECK(mbrtum_process_response_with_request_adu(
              &request,
              request_adu,
              request_length,
              response_adu,
              response_length,
              &response_view) == MBRTUM_ERROR_ACKNOWLEDGEMENT_MISMATCH);
    memcpy(response_adu, request_adu, request_length);

    CHECK(mbrtum_build_get_comm_event_counter_request(
              1u, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    response_adu[0] = 1u;
    response_adu[1] = MBRTUM_FC_GET_COMM_EVENT_COUNTER;
    response_adu[2] = 0xFFu;
    response_adu[3] = 0xFFu;
    response_adu[4] = 0x01u;
    response_adu[5] = 0x08u;
    response_length = append_crc(response_adu, 6u);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response_view) == MBRTUM_OK);
    CHECK(mbrtum_get_comm_event_counter(&response_view,
                                        &status,
                                        &count) == MBRTUM_OK);
    CHECK(status == 0xFFFFu && count == 0x0108u);

    response_length = append_crc(response_adu, 5u);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response_view) ==
          MBRTUM_ERROR_RESPONSE_LENGTH);

    CHECK(mbrtum_build_get_comm_event_log_request(
              1u, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    response_adu[0] = 1u;
    response_adu[1] = MBRTUM_FC_GET_COMM_EVENT_LOG;
    response_adu[2] = 8u;
    response_adu[3] = 0x00u;
    response_adu[4] = 0x00u;
    response_adu[5] = 0x01u;
    response_adu[6] = 0x08u;
    response_adu[7] = 0x01u;
    response_adu[8] = 0x21u;
    response_adu[9] = 0x20u;
    response_adu[10] = 0x00u;
    response_length = append_crc(response_adu, 11u);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response_view) == MBRTUM_OK);
    CHECK(mbrtum_get_comm_event_log(&response_view, &event_log) == MBRTUM_OK);
    CHECK(event_log.event_count == 0x0108u);
    CHECK(event_log.message_count == 0x0121u);
    CHECK(event_log.events_length == 2u);
    CHECK(mbrtum_get_diagnostic_event(&event_log, 0u, &event) == MBRTUM_OK);
    CHECK(event == 0x20u);

    response_adu[2] = 7u;
    response_length = append_crc(response_adu, 11u);
    CHECK(mbrtum_process_response(&request,
                                  response_adu,
                                  response_length,
                                  &response_view) ==
          MBRTUM_ERROR_RESPONSE_LENGTH);
    return EXIT_SUCCESS;
}

static int tx_accept(void *context, const uint8_t *adu, size_t adu_length)
{
    size_t *captured_length = context;

    CHECK(adu != NULL);
    *captured_length = adu_length;
    return MBRTUM_TXN_TRANSMIT_ACCEPTED;
}

static int test_transaction_support(void)
{
    mbrtum_transaction_t transaction;
    const mbrtum_transaction_config_t config = {100u, 0u, 0u, 0u};
    mbrtum_request_t request;
    uint8_t request_adu[MODBUS_RTU_ADU_MAX_SIZE];
    uint8_t response_adu[8];
    size_t request_length = 0u;
    size_t response_length;
    size_t captured_length = 0u;

    CHECK(mbrtum_build_get_comm_event_counter_request(
              1u, &request, request_adu, sizeof(request_adu),
              &request_length) == MBRTUM_OK);
    CHECK(mbrtum_transaction_init(&transaction,
                                  &config,
                                  tx_accept,
                                  &captured_length) == MBRTUM_TXN_OK);
    CHECK(mbrtum_transaction_start(&transaction,
                                   &request,
                                   request_adu,
                                   request_length,
                                   0u) == MBRTUM_TXN_OK);
    CHECK(captured_length == request_length);
    CHECK(mbrtum_transaction_on_tx_complete(&transaction, 1u) ==
          MBRTUM_TXN_OK);

    response_adu[0] = 1u;
    response_adu[1] = MBRTUM_FC_GET_COMM_EVENT_COUNTER;
    response_adu[2] = 0u;
    response_adu[3] = 0u;
    response_adu[4] = 0u;
    response_adu[5] = 7u;
    response_length = append_crc(response_adu, 6u);
    CHECK(mbrtum_transaction_on_response(&transaction,
                                         response_adu,
                                         response_length,
                                         2u) == MBRTUM_TXN_OK);
    CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
    CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);

    {
        const uint8_t query_data[] = {0xA5u, 0x37u};

        captured_length = 0u;
        CHECK(mbrtum_build_diagnostics_request(
                  1u,
                  MBRTU_DIAG_SUB_RETURN_QUERY_DATA,
                  query_data,
                  sizeof(query_data),
                  &request,
                  request_adu,
                  sizeof(request_adu),
                  &request_length) == MBRTUM_OK);
        CHECK(mbrtum_transaction_start(&transaction,
                                       &request,
                                       request_adu,
                                       request_length,
                                       10u) == MBRTUM_TXN_OK);
        CHECK(captured_length == request_length);
        CHECK(mbrtum_transaction_on_tx_complete(&transaction, 11u) ==
              MBRTUM_TXN_OK);
        CHECK(mbrtum_transaction_on_response(&transaction,
                                             request_adu,
                                             request_length,
                                             12u) == MBRTUM_TXN_OK);
        CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
        CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    }

    {
        const uint8_t zero_data[] = {0x00u, 0x00u};
        const size_t captured_before_invalid = captured_length;

        CHECK(mbrtum_build_diagnostics_request(
                  1u,
                  MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY,
                  zero_data,
                  sizeof(zero_data),
                  &request,
                  request_adu,
                  sizeof(request_adu),
                  &request_length) == MBRTUM_OK);
        CHECK(request.expects_response == 0u);

        request_adu[5] = 1u;
        request_length = append_crc(request_adu, request_length - 2u);
        CHECK(mbrtum_transaction_start(&transaction,
                                       &request,
                                       request_adu,
                                       request_length,
                                       20u) == MBRTUM_TXN_ERROR_REQUEST);
        CHECK(captured_length == captured_before_invalid);

        CHECK(mbrtum_build_diagnostics_request(
                  1u,
                  MBRTU_DIAG_SUB_FORCE_LISTEN_ONLY,
                  zero_data,
                  sizeof(zero_data),
                  &request,
                  request_adu,
                  sizeof(request_adu),
                  &request_length) == MBRTUM_OK);
        captured_length = 0u;
        CHECK(mbrtum_transaction_start(&transaction,
                                       &request,
                                       request_adu,
                                       request_length,
                                       21u) == MBRTUM_TXN_OK);
        CHECK(captured_length == request_length);
        CHECK(mbrtum_transaction_on_tx_complete(&transaction, 22u) ==
              MBRTUM_TXN_OK);
        CHECK(transaction.state == MBRTUM_TXN_STATE_COMPLETE);
        CHECK(transaction.result == MBRTUM_TXN_RESULT_SUCCESS);
    }
    return EXIT_SUCCESS;
}

static size_t make_tcp_request(uint8_t function, uint8_t *adu)
{
    adu[0] = 0x12u;
    adu[1] = 0x34u;
    adu[2] = 0u;
    adu[3] = 0u;
    adu[4] = 0u;
    adu[5] = 2u;
    adu[6] = 1u;
    adu[7] = function;
    return 8u;
}

static int test_tcp_rejects_serial_only_functions(void)
{
    const uint8_t functions[] = {
        MBRTU_FC_READ_EXCEPTION_STATUS,
        MBRTU_FC_DIAGNOSTICS,
        MBRTU_FC_GET_COMM_EVENT_COUNTER,
        MBRTU_FC_GET_COMM_EVENT_LOG
    };
    uint8_t request[MODBUS_TCP_ADU_MAX_SIZE];
    uint8_t response[MODBUS_TCP_ADU_MAX_SIZE];

    for (size_t i = 0u; i < sizeof(functions); ++i) {
        size_t request_length = make_tcp_request(functions[i], request);
        size_t response_length = 0u;

        CHECK(mbtcp_process_adu(request,
                                request_length,
                                response,
                                sizeof(response),
                                &response_length) == 0);
        CHECK(response_length == 9u);
        CHECK(response[7] == (uint8_t)(functions[i] | 0x80u));
        CHECK(response[8] == MBRTUM_EXCEPTION_ILLEGAL_FUNCTION);
    }
    return EXIT_SUCCESS;
}

int main(void)
{
    CHECK(test_timing_layer_diagnostics_integration() == EXIT_SUCCESS);
    CHECK(test_slave_read_functions_and_lengths() == EXIT_SUCCESS);
    CHECK(test_fc08_policy_listen_only_and_reset() == EXIT_SUCCESS);
    CHECK(test_broadcast_and_event_log_overflow() == EXIT_SUCCESS);
    CHECK(test_fc08_counters_clear_and_saturation() == EXIT_SUCCESS);
    CHECK(test_master_builders_and_decoders() == EXIT_SUCCESS);
    CHECK(test_transaction_support() == EXIT_SUCCESS);
    CHECK(test_tcp_rejects_serial_only_functions() == EXIT_SUCCESS);
    puts("modbus RTU diagnostics tests: PASS");
    return EXIT_SUCCESS;
}
