#pragma once

namespace bmp390::params
{

// from datasheet
static constexpr uint8_t device_id = 0x60;

static constexpr uint8_t default_i2c_address = 0x76u;   // make sure SDO is connected to GND (address->0x77 if SDO is connected to Vcc)

// registers
static constexpr uint8_t chip_id_reg = 0x00;
static constexpr uint8_t rev_id_reg  = 0x01;

static constexpr uint8_t err_reg              = 0x02;
static constexpr uint8_t status_reg           = 0x03;
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

static constexpr uint8_t pwr_ctrl_reg         = 0x1b;
static constexpr uint8_t osr_reg              = 0x1c;
static constexpr uint8_t odr_reg              = 0x1d;
static constexpr uint8_t config_reg           = 0x1f;
static constexpr uint8_t calibration_data_reg = 0x31;
static constexpr uint8_t cmd_reg              = 0x7e;

// sensor specs
static constexpr float temp_min_plauisble_range = -40.0f;
static constexpr float temp_max_plauisble_range = 85.0f;

static constexpr float pressure_min_plauisble_range = 30'000.0f;
static constexpr float pressure_max_plauisble_range = 125'000.0f;

// register values
struct ErrReg
{
   static constexpr uint8_t fatal_err_mask = 1u << 0;
   static constexpr uint8_t cmd_err_mask   = 1u << 1u;
   static constexpr uint8_t conf_err_mask  = 1u << 2u;
};

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

struct OsrReg
{
   // osr_p
   static constexpr uint8_t press_no_oversampling_mask  = 0;
   static constexpr uint8_t press_x2_oversampling_mask  = 1u;
   static constexpr uint8_t press_x4_oversampling_mask  = 2u;
   static constexpr uint8_t press_x8_oversampling_mask  = 3u;
   static constexpr uint8_t press_x16_oversampling_mask = 4u;
   static constexpr uint8_t press_x32_oversampling_mask = 5u;

   // osr4_t
   static constexpr uint8_t temp_no_oversampling_mask  = 0;
   static constexpr uint8_t temp_x2_oversampling_mask  = 1u << 3u;
   static constexpr uint8_t temp_x4_oversampling_mask  = 2u << 3u;
   static constexpr uint8_t temp_x8_oversampling_mask  = 3u << 3u;
   static constexpr uint8_t temp_x16_oversampling_mask = 4u << 3u;
   static constexpr uint8_t temp_x32_oversampling_mask = 5u << 3u;
};

struct OdrReg
{
   static constexpr uint8_t odr_200hz = 0;
   static constexpr uint8_t odr_100hz = 1u;
   static constexpr uint8_t odr_50hz  = 2u;
   static constexpr uint8_t odr_25hz  = 3u;
   // ...
};

struct ConfigReg
{
   static constexpr uint8_t iir_coef0_mask = 0 << 1u;
   static constexpr uint8_t iir_coef1_mask = 1u << 1u;
   static constexpr uint8_t iir_coef3_mask = 2u << 1u;
   static constexpr uint8_t iir_coef7_mask = 3u << 1u;
   // ...
};

struct CalibrationCoeffiecients
{
   // temperature trimming coefficients
   uint16_t par_t1;
   uint16_t par_t2;
   int8_t   par_t3;

   // pressure trimming coefficients
   int16_t  par_p1;
   int16_t  par_p2;
   int8_t   par_p3;
   int8_t   par_p4;
   uint16_t par_p5;
   uint16_t par_p6;
   int8_t   par_p7;
   int8_t   par_p8;
   int16_t  par_p9;
   int8_t   par_p10;
   int8_t   par_p11;
};

struct QuantizedCoeffiecients
{
   // temperature trimming coefficients
   float par_t1;
   float par_t2;
   float par_t3;

   // pressure trimming coefficients
   double par_p1;
   double par_p2;
   double par_p3;
   double par_p4;
   double par_p5;
   double par_p6;
   double par_p7;
   double par_p8;
   double par_p9;
   double par_p10;
   double par_p11;
};

static constexpr uint8_t num_bytes_data             = 8u;   // 1 byte err_reg + 1 byte status_reg + 3 bytes pressure + 3 bytes temperature
static constexpr uint8_t num_bytes_calibration_data = 21u;
static constexpr uint8_t buffer_size                = 32;

}   // namespace bmp390::params
