#pragma once

#include <cstdint>

namespace mpu6500::params
{

// from datasheet
static constexpr uint8_t device_id = 0x70;

// registers
static constexpr uint8_t self_test_x_gyro_reg  = 0x00;
static constexpr uint8_t self_test_y_gyro_reg  = 0x01;
static constexpr uint8_t self_test_z_gyro_reg  = 0x02;
static constexpr uint8_t self_test_x_accel_reg = 0x0d;
static constexpr uint8_t self_test_y_accel_reg = 0x0e;
static constexpr uint8_t self_test_z_accel_reg = 0x0f;

static constexpr uint8_t smplrt_div_reg   = 0x19;
static constexpr uint8_t cnfg_reg         = 0x1a;
static constexpr uint8_t gyro_cnfg_reg    = 0x1b;
static constexpr uint8_t accel_cnfg_reg   = 0x1c;
static constexpr uint8_t accel_cnfg_2_reg = 0x1d;

static constexpr uint8_t accel_xout_h_reg = 0x3b;
static constexpr uint8_t accel_xout_l_reg = 0x3c;
static constexpr uint8_t accel_yout_h_reg = 0x3d;
static constexpr uint8_t accel_yout_l_reg = 0x3e;
static constexpr uint8_t accel_zout_h_reg = 0x3f;
static constexpr uint8_t accel_zout_l_reg = 0x40;
static constexpr uint8_t temp_out_h_reg   = 0x41;
static constexpr uint8_t temp_out_l_reg   = 0x42;
static constexpr uint8_t gyro_xout_h_reg  = 0x43;
static constexpr uint8_t gyro_xout_l_reg  = 0x44;
static constexpr uint8_t gyro_yout_h_reg  = 0x45;
static constexpr uint8_t gyro_yout_l_reg  = 0x46;
static constexpr uint8_t gyro_zout_h_reg  = 0x47;
static constexpr uint8_t gyro_zout_l_reg  = 0x48;

static constexpr uint8_t signal_path_reset_reg = 0x68;
static constexpr uint8_t user_ctrl_reg         = 0x6a;
static constexpr uint8_t pwr_mgmt_1_reg        = 0x6b;
static constexpr uint8_t who_am_i_reg          = 0x75;

// sensor specs
static constexpr uint16_t gyro_abs_full_scale_range_dps[4u] = {250u, 500u, 1000u, 2000u};            // degrees/s
static constexpr uint16_t accel_abs_full_scale_range_g[4u]  = {2u, 4u, 8u, 16u};                     // g

static constexpr float gyro_sensitivity_scale_factor[4u]  = {131.0f, 65.5f, 32.8f, 16.4f};           // LSB/(degrees/s)
static constexpr float accel_sensitivity_scale_factor[4u] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};   // LSB/g
static constexpr float temp_sensitivity                   = 333.87f;                                 // LSB/Degree-C
static constexpr float temp_offset                        = 21.0f;                                   // LSB

static constexpr float temp_min_plauisble_range = -25.0f;
static constexpr float temp_max_plauisble_range = 85.0f;

// register value bit masks
static constexpr uint8_t read_mask = 0x80;

struct SampleRateDividerBitMask
{
   static constexpr uint8_t smplrt_div = 0b1111'1111;
};

struct ConfigBitMask
{
   static constexpr uint8_t dlpf_cfg = 0b0000'0111;
};

struct GyroConfigBitMask
{
   static constexpr uint8_t gyro_fs_sel = 0b0001'1000;
   static constexpr uint8_t f_choice_b  = 0b0000'0011;
};

struct AccelConfigBitMask
{
   static constexpr uint8_t accel_fs_sel = 0b0001'1000;
};

struct AccelConfig2BitMask
{
   static constexpr uint8_t accel_f_choice_b    = 0b0000'1000;
   static constexpr uint8_t accel_a_dlpf_config = 0b0000'0111;
};

struct PwrMgmt1BitMask
{
   static constexpr uint8_t device_reset = 0b1000'0000;
   static constexpr uint8_t clock_sel    = 0b0000'0001;
};

struct SignalPathResetBitMask
{
   static constexpr uint8_t gyro_reset  = 0b0000'0100;
   static constexpr uint8_t accel_reset = 0b0000'0010;
   static constexpr uint8_t temp_reset  = 0b0000'0001;
};

struct UserCtrlBitMask
{
   static constexpr uint8_t i2c_if_dis = 0b0001'0000;
};

// wait duration
static constexpr uint8_t power_on_reset_wait_ms    = 100;
static constexpr uint8_t signal_path_reset_wait_ms = 100;

// spi transaction bytes
static constexpr uint8_t num_bytes_register      = 1u;
static constexpr uint8_t num_bytes_accelerometer = 6u;
static constexpr uint8_t num_bytes_gyroscope     = 6u;
static constexpr uint8_t num_bytes_temperature   = 2u;
static constexpr uint8_t num_bytes_transaction   = num_bytes_register + num_bytes_accelerometer + num_bytes_gyroscope + num_bytes_temperature;

}   // namespace mpu6500::params
