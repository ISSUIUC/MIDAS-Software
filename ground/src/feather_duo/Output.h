#pragma once
#include"Packet.h"


static constexpr const char* json_command_success = R"({"type": "command_success"})";
static constexpr const char* json_command_bad = R"({"type": "bad_command"})";
static constexpr const char* json_command_sent = R"({"type": "command_sent"})";
static constexpr const char* json_command_ack = R"({"type": "command_acknowledge"})";
static constexpr const char* json_command_parse_error = R"({"type": "command_error", "error": "serial parse error"})";
static constexpr const char* json_buffer_full_error = R"({"type": "command_error", "error": "command buffer not empty"})";

static constexpr const char* json_init_failure = R"({"type": "init_error", "error": "failed to initilize LORA"})";
static constexpr const char* json_init_success = R"({"type": "init_success"})";
static constexpr const char* json_set_frequency_failure = R"({"type": "freq_error", "error": "set_frequency failed"})";
static constexpr const char* json_receive_failure = R"({"type": "receive_error", "error": "recv failed"})";
static constexpr const char* json_send_failure = R"({"type": "send_error", "error": "command_retries_exceded"})";

void printPacketJson(FullTelemetryData const& packet);