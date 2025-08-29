#pragma once

namespace bmp390::params
{

// from datasheet
static constexpr uint8_t device_id = 0x60;

// registers

static constexpr std::size_t spi_timeout_us = 250u;     // @TODO: confirm

static constexpr uint8_t num_bytes_transaction = 15u;   // @TODO: confirm

}   // namespace bmp390::params
