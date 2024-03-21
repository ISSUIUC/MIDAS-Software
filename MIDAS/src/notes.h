#pragma once
#include <cstdint>
#include "buzzer.h"

#define MS_PER_4BEAT 6000

#define rest Sound{0, 10}
#define d4_eight Sound{294, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
#define g4_eight Sound{392, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
#define f_nat_4_eight Sound{350, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
#define b_flat_4_eight Sound{466, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
#define e4_eight Sound{330, static_cast<uint8_t>(0.125 * MS_PER_4BEAT)}
#define d4_quart Sound{294, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define g4_quart Sound{392, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define f_nat_4_quart Sound{350, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define b_flat_4_quart Sound{466, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define e4_quart Sound{330, static_cast<uint8_t>(0.25 * MS_PER_4BEAT)}
#define d4_fifth Sound{294, static_cast<uint8_t>(0.05 * MS_PER_4BEAT)}
#define f_nat_4_fifth Sound{350, static_cast<uint8_t>(0.05 * MS_PER_4BEAT)}
#define d4_2fifth Sound{294, static_cast<uint8_t>(0.1 * MS_PER_4BEAT)}
#define f_nat_4_2fifth Sound{350, static_cast<uint8_t>(0.1 * MS_PER_4BEAT)}
