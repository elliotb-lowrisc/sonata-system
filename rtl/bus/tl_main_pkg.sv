// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// tl_main package generated by `tlgen.py` tool

package tl_main_pkg;

  localparam logic [31:0] ADDR_SPACE_SRAM      = 32'h 00100000;
  localparam logic [31:0] ADDR_SPACE_GPIO      = 32'h 80000000;
  localparam logic [31:0] ADDR_SPACE_PWM       = 32'h 80001000;
  localparam logic [31:0] ADDR_SPACE_TIMER     = 32'h 80040000;
  localparam logic [31:0] ADDR_SPACE_UART      = 32'h 80100000;
  localparam logic [31:0] ADDR_SPACE_I2C0      = 32'h 80200000;
  localparam logic [31:0] ADDR_SPACE_I2C1      = 32'h 80201000;
  localparam logic [31:0] ADDR_SPACE_SPI_FLASH = 32'h 80300000;
  localparam logic [31:0] ADDR_SPACE_SPI_LCD   = 32'h 80301000;
  localparam logic [31:0] ADDR_SPACE_RV_PLIC   = 32'h 88000000;

  localparam logic [31:0] ADDR_MASK_SRAM      = 32'h 0001ffff;
  localparam logic [31:0] ADDR_MASK_GPIO      = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_PWM       = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_TIMER     = 32'h 0000ffff;
  localparam logic [31:0] ADDR_MASK_UART      = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_I2C0      = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_I2C1      = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_SPI_FLASH = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_SPI_LCD   = 32'h 00000fff;
  localparam logic [31:0] ADDR_MASK_RV_PLIC   = 32'h 03ffffff;

  localparam int N_HOST   = 2;
  localparam int N_DEVICE = 10;

  typedef enum int {
    TlSram = 0,
    TlGpio = 1,
    TlPwm = 2,
    TlTimer = 3,
    TlUart = 4,
    TlI2C0 = 5,
    TlI2C1 = 6,
    TlSpiFlash = 7,
    TlSpiLcd = 8,
    TlRvPlic = 9
  } tl_device_e;

  typedef enum int {
    TlIbexLsu = 0,
    TlDbgHost = 1
  } tl_host_e;

endpackage
