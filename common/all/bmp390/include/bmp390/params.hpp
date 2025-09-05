#pragma once

namespace bmp390::params
{

// from datasheet
static constexpr uint8_t device_id = 0x60;

static constexpr uint8_t default_i2c_address = 0x76u;   // make sure SDO is connected to GND (address->0x77 if SDO is connected to Vcc)

// registers
static constexpr uint8_t chip_id_reg = 0x00;
static constexpr uint8_t rev_id_reg  = 0x01;
static constexpr uint8_t err_reg     = 0x02;
static constexpr uint8_t status_reg  = 0x03;

static constexpr uint8_t pressure_xlsb_reg    = 0x04;
static constexpr uint8_t pressure_lsb_reg     = 0x05;
static constexpr uint8_t pressure_msb_reg     = 0x06;
static constexpr uint8_t temperature_xlsb_reg = 0x07;
static constexpr uint8_t temperature_lsb_reg  = 0x08;
static constexpr uint8_t temperature_msb_reg  = 0x09;

static constexpr uint8_t event_reg      = 0x10;
static constexpr uint8_t int_status_reg = 0x11;
static constexpr uint8_t int_ctrl_reg   = 0x19;
static constexpr uint8_t if_conf_reg    = 0x1a;
static constexpr uint8_t pwr_ctrl_reg   = 0x1b;

static constexpr uint8_t osr_reg    = 0x1c;
static constexpr uint8_t odr_reg    = 0x1d;
static constexpr uint8_t config_reg = 0x1f;
static constexpr uint8_t cmd_reg    = 0x7e;

// register values
struct CmdReg
{
   static constexpr uint8_t nop        = 0x00;
   static constexpr uint8_t fifo_flugh = 0xb0;
   static constexpr uint8_t soft_reset = 0xb6;
};

struct PwrCtrlReg
{
   static constexpr uint8_t pressure_sensor_enable_mask    = 1u << 0;
   static constexpr uint8_t temperature_sensor_enable_mask = 1u << 1u;
   static constexpr uint8_t normal_mode_mask               = 3u << 4u;
};

static constexpr uint8_t num_bytes_data        = 6u;    // 3 bytes pressure + 3 bytes temperature
static constexpr uint8_t num_bytes_transaction = 16u;   // @TODO: confirm

}   // namespace bmp390::params
