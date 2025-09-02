#pragma once

namespace bmp390::params
{

// from datasheet
static constexpr uint8_t device_id = 0x60;

static constexpr uint8_t default_i2c_address = 0x76u;   // make sure SDO is connected to GND

// registers

static constexpr std::size_t spi_timeout_us = 250u;     // @TODO: confirm

static constexpr uint8_t num_bytes_transaction = 16u;   // @TODO: confirm

}   // namespace bmp390::params
