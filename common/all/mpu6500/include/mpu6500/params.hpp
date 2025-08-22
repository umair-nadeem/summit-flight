#pragma once

#include <cstdint>

namespace mpu6500::params
{

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

// bit masks
static constexpr uint8_t read_mask  = 0x80;
static constexpr uint8_t write_mask = 0x7f;

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

// spi transaction bytes
static constexpr uint8_t num_bytes_register      = 1u;
static constexpr uint8_t num_bytes_accelerometer = 6u;
static constexpr uint8_t num_bytes_gyroscope     = 6u;
static constexpr uint8_t num_bytes_temperature   = 2u;
static constexpr uint8_t num_bytes_transaction   = num_bytes_register + num_bytes_accelerometer + num_bytes_gyroscope + num_bytes_temperature + 1u;

// wait duration
static constexpr uint8_t power_on_reset_wait_ms    = 100;
static constexpr uint8_t signal_path_reset_wait_ms = 100;

}   // namespace mpu6500::params
