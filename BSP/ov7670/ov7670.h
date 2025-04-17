/*
 * OV7670.h
 * OV7670 DCMI DMA Driver
 * Created on: Aug 5, 2024
 *     Author: K.Rudenko
 */

#ifndef OV7670_H_
#define OV7670_H_

/******************************************************************************
 *                                 INCLUDES                                   *
 ******************************************************************************/

#include "main.h"

/******************************************************************************
 *                            CONFIGURATION MACRO                             *
 ******************************************************************************/

#define DISPLAY_WIDTH                            (800U)
#define DISPLAY_HEIGHT                           (480U)

#define OV7670_WIDTH                             (320U)
#define OV7670_HEIGHT                            (240U)

#define OV7670_STREAM_MODE_BY_FRAME              0
#define OV7670_STREAM_MODE_BY_LINE               1
#define OV7670_STREAM_MODE                       OV7670_STREAM_MODE_BY_FRAME

/* Delay API */
#define OV7670_DELAY(ms)                         HAL_Delay(ms)

/* GPIO pin mapping */
#define	OV7670_GPIO_PORT_PWDN	DCMI_PWDN_GPIO_Port
#define	OV7670_GPIO_PIN_PWDN 	DCMI_PWDN_Pin
#define	OV7670_GPIO_PORT_RET	CAMERA_RESET_GPIO_Port
#define	OV7670_GPIO_PIN_RET 	CAMERA_RESET_Pin


/* Note: XLK pin shall be connected to the corresponding timer OCU pin */
/* D0..D7, VS, HS, PLK shall be connected to 8-bit with External Sync DCMI */
/* SCCB SCL, SDA - to I2C */
/* For GPIO, ,DMA, Timer configuration - please see in CubeMX project */
/* Example for OV7670 <-> STM32F407VET6 connections:
 * SCL  -> PB10 (I2C2)
 * SDA  -> PB11 (I2C2)
 * VS   -> PB7  (DCMI)
 * HS   -> PA4  (DCMI)
 * PLK  -> PA6  (DCMI)
 * XLK  -> PC2  (TIM5 OCU CH3) 10.5..42 MHz
 * D7   -> PE6  (DCMI)
 * D6   -> PE5  (DCMI)
 * D5   -> PB6  (DCMI)
 * D4   -> PE4  (DCMI)
 * D3   -> PC9  (DCMI)
 * D2   -> PC8  (DCMI)
 * D1   -> PC7  (DCMI)
 * D0   -> PC6  (DCMI)
 * RET  -> PD11 (GPIO)
 * PWDN -> PD12 (GPIO)
 */

/* Example of XLK clock signal configuration:
 * TIM5 CH3 OCU configuration (please see in CubeMx):
 * Mode:                     Ooutput Compare CH3, Toggle on Match;
 * Pin:                      PA2;
 * TIM_CLK (clock source):   84MHz,
 * TIM_CLD (clock division): 1,
 * TIM_PSC (prescaler - 1):  0,
 * TIM_CNT_CLK:              84000000,
 * TIM_ARR (period - 1):     1,
 * Pulse:                    1,
 * Polarity:                 any,
 * => f = 42MHz (max possible and max for OV7670 XLK).
 */

/* TIM_CNT_CLK = TIM_CLK / TIM_CLD /(TIM_PSC + 1);
 * f  = TIM_CLK / (TIM_PSC + 1)/(TIM_ARR + 1);
 * Pulse = TIM_CNT_CLK / f / 2;
 */

/******************************************************************************
 *                           OV7670 REGISTERS                                 *
 ******************************************************************************/
#define OV7670_REG_GAIN                (0x0U)   /* Def: 0x0; R/W: RW; AGC – Gain control gain setting */
#define OV7670_REG_BLUE                (0x1U)   /* Def: 0x80; R/W: RW; AWB – Blue channel gain setting */
#define OV7670_REG_RED                 (0x2U)   /* Def: 0x80; R/W: RW; AWB – Red channel gain setting */
#define OV7670_REG_VREF                (0x3U)   /* Def: 0x0; R/W: RW; Vertical Frame Control */
#define OV7670_REG_COM1                (0x4U)   /* Def: 0x0; R/W: RW; Common Control 1 */
#define OV7670_REG_BAVE                (0x5U)   /* Def: 0x0; R/W: RW; U/B Average Level */
#define OV7670_REG_GbAVE               (0x6U)   /* Def: 0x0; R/W: RW; Y/Gb Average Level */
#define OV7670_REG_AECHH               (0x7U)   /* Def: 0x0; R/W: RW; Exposure Value - AEC MSB 5 bits */
#define OV7670_REG_RAVE                (0x8U)   /* Def: 0x0; R/W: RW; V/R Average Level */
#define OV7670_REG_COM2                (0x9U)   /* Def: 0x1; R/W: RW; Common Control 2  */
#define OV7670_REG_PID                 (0x0AU)  /* Def: 0x76; R/W: R; Product ID Number MSB (Read only) */
#define OV7670_REG_VER                 (0x0BU)  /* Def: 0x73; R/W: R; Product ID Number LSB (Read only) */
#define OV7670_REG_COM3                (0x0CU)  /* Def: 0x0; R/W: RW; Common Control 3 */
#define OV7670_REG_COM4                (0x0DU)  /* Def: 0x0; R/W: RW; Common Control 4 */
#define OV7670_REG_COM5                (0x0EU)  /* Def: 0x1; R/W: RW; Common Control 5 */
#define OV7670_REG_COM6                (0x0FU)  /* Def: 0x43; R/W: RW; Common Control 6 */
#define OV7670_REG_AECH                (0x10U)  /* Def: 0x40; R/W: RW; Exposure Value */
#define OV7670_REG_CLKRC               (0x11U)  /* Def: 0x80; R/W: RW; Internal Clock */
#define OV7670_REG_COM7                (0x12U)  /* Def: 0x0; R/W: RW; Common Control 7 */
#define OV7670_REG_COM8                (0x13U)  /* Def: 0x8F; R/W: RW; Common Control 8 */
#define OV7670_REG_COM9                (0x14U)  /* Def: 0x4A; R/W: RW; Common Control 9 */
#define OV7670_REG_COM10               (0x15U)  /* Def: 0x0; R/W: RW; Common Control 10 */
#define OV7670_REG_HSTART              (0x17U)  /* Def: 0x11; R/W: RW; Output Format - Horizontal Frame (HREF column) start high 8-bit (low 3 bits are at HREF[2:0]) */
#define OV7670_REG_HSTOP               (0x18U)  /* Def: 0x61; R/W: RW; Output Format - Horizontal Frame (HREF column) end high 8-bit (low 3 bits are at HREF[5:3]) */
#define OV7670_REG_VSTRT               (0x19U)  /* Def: 0x3; R/W: RW; Output Format - Vertical Frame (row) start high 8-bit (low 2 bits are at VREF[1:0]) */
#define OV7670_REG_VSTOP               (0x1AU)  /* Def: 0x7B; R/W: RW; Output Format - Vertical Frame (row) end high 8-bit (low 2 bits are at VREF[3:2]) */
#define OV7670_REG_PSHFT               (0x1BU)  /* Def: 0x0; R/W: RW; Data Format - Pixel Delay Select (delays timing of the D[7:0] data relative to HREF in pixel units) */
#define OV7670_REG_MIDH                (0x1CU)  /* Def: 0x7F; R/W: R; Manufacturer ID Byte – High (Read only = 0x7F) */
#define OV7670_REG_MIDL                (0x1DU)  /* Def: 0xA2; R/W: R; Manufacturer ID Byte – Low (Read only = 0xA2) */
#define OV7670_REG_MVFP                (0x1EU)  /* Def: 0x1; R/W: RW; Mirror/VFlip Enable */
#define OV7670_REG_LAEC                (0x1FU)  /* Def: 0x0; R/W: RW; Reserved */
#define OV7670_REG_ADCCTR0             (0x20U)  /* Def: 0x4; R/W: RW; ADC Control */
#define OV7670_REG_ADCCTR1             (0x21U)  /* Def: 0x2; R/W: RW; Bit[7:0]: Reserved */
#define OV7670_REG_ADCCTR2             (0x22U)  /* Def: 0x1; R/W: RW; Bit[7:0]: Reserved */
#define OV7670_REG_ADCCTR3             (0x23U)  /* Def: 0x0; R/W: RW; Bit[7:0]: Reserved */
#define OV7670_REG_AEW                 (0x24U)  /* Def: 0x75; R/W: RW; AGC/AEC - Stable Operating Region (Upper Limit) */
#define OV7670_REG_AEB                 (0x25U)  /* Def: 0x63; R/W: RW; AGC/AEC - Stable Operating Region (Lower Limit) */
#define OV7670_REG_VPT                 (0x26U)  /* Def: 0xD4; R/W: RW; AGC/AEC Fast Mode Operating Region */
#define OV7670_REG_BBIAS               (0x27U)  /* Def: 0x80; R/W: RW; B Channel Signal Output Bias (effective only when COM6[3] = 1)  */
#define OV7670_REG_GbBIAS              (0x28U)  /* Def: 0x80; R/W: RW; Gb Channel Signal Output Bias (effective only when COM6[3] = 1) */
#define OV7670_REG_EXHCH               (0x2AU)  /* Def: 0x0; R/W: RW; Dummy Pixel Insert MSB */
#define OV7670_REG_EXHCL               (0x2BU)  /* Def: 0x0; R/W: RW; Dummy Pixel Insert LSB */
#define OV7670_REG_RBIAS               (0x2CU)  /* Def: 0x80; R/W: RW; R Channel Signal Output Bias (effective only when COM6[3] = 1) */
#define OV7670_REG_ADVFL               (0x2DU)  /* Def: 0x0; R/W: RW; LSB of Insert Dummy Lines in Vertical Direction (1 bit equals 1 line) */
#define OV7670_REG_ADVFH               (0x2EU)  /* Def: 0x0; R/W: RW; MSB of Insert Dummy Lines in Vertical Direction */
#define OV7670_REG_YAVE                (0x2FU)  /* Def: 0x0; R/W: RW; Y/G Channel Average Value */
#define OV7670_REG_HSYST               (0x30U)  /* Def: 0x8; R/W: RW; HSYNC Rising Edge Delay (low 8 bits) */
#define OV7670_REG_HSYEN               (0x31U)  /* Def: 0x30; R/W: RW; HSYNC Falling Edge Delay (low 8 bits) */
#define OV7670_REG_HREF                (0x32U)  /* Def: 0x80; R/W: RW; HREF Control */
#define OV7670_REG_CHLF                (0x33U)  /* Def: 0x8; R/W: RW; Array Current Control */
#define OV7670_REG_ARBLM               (0x34U)  /* Def: 0x11; R/W: RW; Array Reference Control */
#define OV7670_REG_ADC                 (0x37U)  /* Def: 0x3F; R/W: RW; ADC Control */
#define OV7670_REG_ACOM                (0x38U)  /* Def: 0x1; R/W: RW; ADC and Analog Common Mode Control */
#define OV7670_REG_OFON                (0x39U)  /* Def: 0x0; R/W: RW; ADC Offset Control */
#define OV7670_REG_TSLB                (0x3AU)  /* Def: 0x0D; R/W: RW; Line Buffer Test Option */
#define OV7670_REG_COM11               (0x3BU)  /* Def: 0x0; R/W: RW; Common Control 11 */
#define OV7670_REG_COM12               (0x3CU)  /* Def: 0x68; R/W: RW; Common Control 12 */
#define OV7670_REG_COM13               (0x3DU)  /* Def: 0x88; R/W: RW; Common Control 13 */
#define OV7670_REG_COM14               (0x3EU)  /* Def: 0x0; R/W: RW; Common Control 14 */
#define OV7670_REG_EDGE                (0x3FU)  /* Def: 0x0; R/W: RW; Edge Enhancement Adjustment */
#define OV7670_REG_COM15               (0x40U)  /* Def: 0xC0; R/W: RW; Common Control 15 */
#define OV7670_REG_COM16               (0x41U)  /* Def: 0x8; R/W: RW; Common Control 16 */
#define OV7670_REG_COM17               (0x42U)  /* Def: 0x0; R/W: RW; Common Control 17 */
#define OV7670_REG_AWBC1               (0x43U)  /* Def: 0x14; R/W: RW; Reserved */
#define OV7670_REG_AWBC2               (0x44U)  /* Def: 0xF0; R/W: RW; Reserved */
#define OV7670_REG_AWBC3               (0x45U)  /* Def: 0x45; R/W: RW; Reserved */
#define OV7670_REG_AWBC4               (0x46U)  /* Def: 0x61; R/W: RW; Reserved */
#define OV7670_REG_AWBC5               (0x47U)  /* Def: 0x51; R/W: RW; Reserved */
#define OV7670_REG_AWBC6               (0x48U)  /* Def: 0x79; R/W: RW; Reserved */
#define OV7670_REG_REG4B               (0x4BU)  /* Def: 0x0; R/W: RW; Register 4B */
#define OV7670_REG_DNSTH               (0x4CU)  /* Def: 0x0; R/W: RW; De-noise Strength */
#define OV7670_REG_MTX1                (0x4FU)  /* Def: 0x40; R/W: RW; Matrix Coefficient 1 */
#define OV7670_REG_MTX2                (0x50U)  /* Def: 0x34; R/W: RW; Matrix Coefficient 2 */
#define OV7670_REG_MTX3                (0x51U)  /* Def: 0x0C; R/W: RW; Matrix Coefficient 3 */
#define OV7670_REG_MTX4                (0x52U)  /* Def: 0x17; R/W: RW; Matrix Coefficient 4 */
#define OV7670_REG_MTX5                (0x53U)  /* Def: 0x29; R/W: RW; Matrix Coefficient 5 */
#define OV7670_REG_MTX6                (0x54U)  /* Def: 0x40; R/W: RW; Matrix Coefficient 6 */
#define OV7670_REG_BRIGHT              (0x55U)  /* Def: 0x0; R/W: RW; Brightness Control */
#define OV7670_REG_CONTRAS             (0x56U)  /* Def: 0x40; R/W: RW; Contrast Control */
#define OV7670_REG_CONTRAS_CENTER      (0x57U)  /* Def: 0x80; R/W: RW; Contrast Center */
#define OV7670_REG_MTXS                (0x58U)  /* Def: 0x1E; R/W: RW; Matrix Coefficient Sign for Coefficient 5 to 0 */
#define OV7670_REG_LCC1                (0x62U)  /* Def: 0x0; R/W: RW; Lens Correction Option 1 - X Coordinate of Lens Correction Center Relative to Array Center */
#define OV7670_REG_LCC2                (0x63U)  /* Def: 0x0; R/W: RW; Lens Correction Option 2 - Y Coordinate of Lens Correction Center Relative to Array Center */
#define OV7670_REG_LCC3                (0x64U)  /* Def: 0x50; R/W: RW; Lens Correction Option 3 */
#define OV7670_REG_LCC4                (0x65U)  /* Def: 0x30; R/W: RW; Lens Correction Option 4 - Radius of the circular section where no compensation applies */
#define OV7670_REG_LCC5                (0x66U)  /* Def: 0x0; R/W: RW; Lens Correction Control 5 */
#define OV7670_REG_MANU                (0x67U)  /* Def: 0x80; R/W: RW; Manual U Value (effective only when register TSLB[4] is high) */
#define OV7670_REG_MANV                (0x68U)  /* Def: 0x80; R/W: RW; Manual V Value (effective only when register TSLB[4] is high) */
#define OV7670_REG_GFIX                (0x69U)  /* Def: 0x0; R/W: RW; Fix Gain Control */
#define OV7670_REG_GGAIN               (0x6AU)  /* Def: 0x0; R/W: RW; G Channel AWB Gain */
#define OV7670_REG_DBLV                (0x6BU)  /* Def: 0x0A; R/W: RW; PLL control */
#define OV7670_REG_AWBCTR3             (0x6CU)  /* Def: 0x2; R/W: RW; AWB Control 3 */
#define OV7670_REG_AWBCTR2             (0x6DU)  /* Def: 0x55; R/W: RW; AWB Control 2 */
#define OV7670_REG_AWBCTR1             (0x6EU)  /* Def: 0xC0; R/W: RW; AWB Control 1 */
#define OV7670_REG_AWBCTR0             (0x6FU)  /* Def: 0x9A; R/W: RW; AWB Control 0 */
#define OV7670_REG_SCALING_XSC         (0x70U)  /* Def: 0x3A; R/W: RW; Bit[7]: Test_pattern[0] - works with test_pattern[1] test_pattern (SCALING_XSC[7], SCALING_YSC[7]): */
#define OV7670_REG_SCALING_YSC         (0x71U)  /* Def: 0x35; R/W: RW; Bit[7]: Test_pattern[1] - works with test_pattern[0] test_pattern (SCALING_XSC[7], SCALING_YSC[7]): */
#define OV7670_REG_SCALING_DCWCTR      (0x72U)  /* Def: 0x11; R/W: RW; DCW Control */
#define OV7670_REG_SCALING_PCLK_DIV    (0x73U)  /* Def: 0x0; R/W: RW; Register 73 */
#define OV7670_REG_REG74               (0x74U)  /* Def: 0x0; R/W: RW; Register 74 */
#define OV7670_REG_REG75               (0x75U)  /* Def: 0x0F; R/W: RW; Register 75 */
#define OV7670_REG_REG76               (0x76U)  /* Def: 0x1; R/W: RW; Register 76 */
#define OV7670_REG_REG77               (0x77U)  /* Def: 0x10; R/W: RW; Register 77 */
#define OV7670_REG_SLOP                (0x7AU)  /* Def: 0x24; R/W: RW; Gamma Curve Highest Segment Slope - calculated as follows: SLOP[7:0] = (0x100 - GAM15[7:0]) x 4/3 */
#define OV7670_REG_GAM1                (0x7BU)  /* Def: 0x4; R/W: RW; Gamma Curve 1st Segment Input End Point 0x04 Output Value */
#define OV7670_REG_GAM2                (0x7CU)  /* Def: 0x7; R/W: RW; Gamma Curve 2nd Segment Input End Point 0x08 Output Value */
#define OV7670_REG_GAM3                (0x7DU)  /* Def: 0x10; R/W: RW; Gamma Curve 3rd Segment Input End Point 0x10 Output Value */
#define OV7670_REG_GAM4                (0x7EU)  /* Def: 0x28; R/W: RW; Gamma Curve 4th Segment Input End Point 0x20 Output Value */
#define OV7670_REG_GAM5                (0x7FU)  /* Def: 0x36; R/W: RW; Gamma Curve 5th Segment Input End Point 0x28 Output Value */
#define OV7670_REG_GAM6                (0x80U)  /* Def: 0x44; R/W: RW; Gamma Curve 6th Segment Input End Point 0x30 Output Value */
#define OV7670_REG_GAM7                (0x81U)  /* Def: 0x52; R/W: RW; Gamma Curve 7th Segment Input End Point 0x38 Output Value */
#define OV7670_REG_GAM8                (0x82U)  /* Def: 0x60; R/W: RW; Gamma Curve 8th Segment Input End Point 0x40 Output Value */
#define OV7670_REG_GAM9                (0x83U)  /* Def: 0x6C; R/W: RW; Gamma Curve 9th Segment Input End Point 0x48 Output Value */
#define OV7670_REG_GAM10               (0x84U)  /* Def: 0x78; R/W: RW; Gamma Curve 10th Segment Input End Point 0x50 Output Value */
#define OV7670_REG_GAM11               (0x85U)  /* Def: 0x8C; R/W: RW; Gamma Curve 11th Segment Input End Point 0x60 Output Value */
#define OV7670_REG_GAM12               (0x86U)  /* Def: 0x9E; R/W: RW; Gamma Curve 12th Segment Input End Point 0x70 Output Value */
#define OV7670_REG_GAM13               (0x87U)  /* Def: 0xBB; R/W: RW; Gamma Curve 13th Segment Input End Point 0x90 Output Value */
#define OV7670_REG_GAM14               (0x88U)  /* Def: 0xD2; R/W: RW; Gamma Curve 14th Segment Input End Point 0xB0 Output Value */
#define OV7670_REG_GAM15               (0x89U)  /* Def: 0xE5; R/W: RW; Gamma Curve 15th Segment Input End Point 0xD0 Output Value */
#define OV7670_REG_RGB444              (0x8CU)  /* Def: 0x0; R/W: RW; Register 8C */
#define OV7670_REG_DM_LNL              (0x92U)  /* Def: 0x0; R/W: RW; Dummy Line low 8 bits */
#define OV7670_REG_DM_LNH              (0x93U)  /* Def: 0x0; R/W: RW; Dummy Line high 8 bits */
#define OV7670_REG_LCC6                (0x94U)  /* Def: 0x50; R/W: RW; Lens Correction Option 6 (effective only when LCC5[2] is high) */
#define OV7670_REG_LCC7                (0x95U)  /* Def: 0x50; R/W: RW; Lens Correction Option 7 (effective only when LCC5[2] is high) */
#define OV7670_REG_BD50ST              (0x9DU)  /* Def: 0x99; R/W: RW; 50 Hz Banding Filter Value */
#define OV7670_REG_BD60ST              (0x9EU)  /* Def: 0x7F; R/W: RW; 60 Hz Banding Filter Value */
#define OV7670_REG_HAECC1              (0x9FU)  /* Def: 0xC0; R/W: RW; Histogram-based AEC/AGC Control 1 */
#define OV7670_REG_HAECC2              (0xA0U)  /* Def: 0x90; R/W: RW; Histogram-based AEC/AGC Control 2 */
#define OV7670_REG_SCALING_PCLK_DELAY  (0xA2U)  /* Def: 0x2; R/W: RW; Pixel Clock Delay */
#define OV7670_REG_NT_CTRL             (0xA4U)  /* Def: 0x0; R/W: RW; Register A4 */
#define OV7670_REG_BD50MAX             (0xA5U)  /* Def: 0x0F; R/W: RW; 50Hz Banding Step Limit */
#define OV7670_REG_HAECC3              (0xA6U)  /* Def: 0xF0; R/W: RW; Histogram-based AEC/AGC Control 3 */
#define OV7670_REG_HAECC4              (0xA7U)  /* Def: 0xC1; R/W: RW; Histogram-based AEC/AGC Control 4 */
#define OV7670_REG_HAECC5              (0xA8U)  /* Def: 0xF0; R/W: RW; Histogram-based AEC/AGC Control 5 */
#define OV7670_REG_HAECC6              (0xA9U)  /* Def: 0xC1; R/W: RW; Histogram-based AEC/AGC Control 6 */
#define OV7670_REG_HAECC7              (0xAAU)  /* Def: 0x14; R/W: RW; Bit[7]: AEC algorithm selection */
#define OV7670_REG_BD60MAX             (0xABU)  /* Def: 0x0F; R/W: RW; 60Hz Banding Step Limit */
#define OV7670_REG_STR_OPT             (0xACU)  /* Def: 0x0; R/W: RW; Register AC */
#define OV7670_REG_STR_R               (0xADU)  /* Def: 0x80; R/W: RW; R Gain for LED Output Frame */
#define OV7670_REG_STR_G               (0xAEU)  /* Def: 0x80; R/W: RW; G Gain for LED Output Frame */
#define OV7670_REG_STR_B               (0xAFU)  /* Def: 0x80; R/W: RW; B Gain for LED Output Frame */
#define OV7670_REG_RSRVD               (0xB0U)  /* */
#define OV7670_REG_ABLC1               (0xB1U)  /* Def: 0x0; R/W: RW; */
#define OV7670_REG_THL_ST              (0xB3U)  /* Def: 0x80; R/W: RW; ABLC Target */
#define OV7670_REG_THL_DLT             (0xB5U)  /* Def: 0x4; R/W: RW; ABLC Stable Range */
#define OV7670_REG_AD_CHB              (0xBEU)  /* Def: 0x0; R/W: RW; Blue Channel Black Level Compensation */
#define OV7670_REG_AD_CHR              (0xBFU)  /* Def: 0x0; R/W: RW; Red Channel Black Level Compensation */
#define OV7670_REG_AD_CHGb             (0xC0U)  /* Def: 0x0; R/W: RW; Gb Channel Black Level Compensation */
#define OV7670_REG_AD_CHGr             (0xC1U)  /* Def: 0x0; R/W: RW; Gr Channel Black Level Compensation */
#define OV7670_REG_SATCTR              (0xC9U)  /* Def: 0xC0; R/W: RW; Saturation Control */
#define OV7670_REG_DUMMY               (0xFFU)

/* OV7670 Registers definition */
#ifndef	OV7670_REG_VER
#define OV7670_AGC        0x00
#define OV7670_BLUE       0x01
#define OV7670_RED        0x02
#define OV7670_VREF       0x03
#define OV7670_COM1       0x04
#define OV7670_BAVE       0x05
#define OV7670_GbAVE      0x06
#define OV7670_AECHH      0x07
#define OV7670_RAVE       0x08
#define OV7670_COM2       0x09
#define OV7670_PID        0x0A
#define OV7670_VER        0x0B
#define OV7670_COM3       0x0C
#define OV7670_COM4       0x0D
#define OV7670_COM5       0x0E
#define OV7670_COM6       0x0F
#define OV7670_AECH       0x10
#define OV7670_CLKRC      0x11
#define OV7670_COM7       0x12
#define OV7670_COM8       0x13
#define OV7670_COM9       0x14
#define OV7670_COM10      0x15
//#define OV7670_RSVD       0x16
#define OV7670_HSTART     0x17
#define OV7670_HSTOP      0x18
#define OV7670_VSTART     0x19
#define OV7670_VSTOP      0x1A
#define OV7670_PSHFT      0x1B
#define OV7670_MIDH       0x1C
#define OV7670_MIDL       0x1D
#define OV7670_MVFP       0x1E
#define OV7670_LAEC		  0x1F
#define OV7670_ADCCTR0    0x20
#define OV7670_ADCCTR1    0x21
#define OV7670_ADCCTR2    0x22
#define OV7670_ADCCTR3    0x23
#define OV7670_AEW        0x24
#define OV7670_AEB        0x25
#define OV7670_VPT        0x26
#define OV7670_BBIAS      0x27
#define OV7670_GbBIAS     0x28
//#define OV7670_RSVD    	  0x29
#define OV7670_EXHCH      0x2A
#define OV7670_EXHCL      0x2B
#define OV7670_RBIAS      0x2C
#define OV7670_ADVFL      0x2D
#define OV7670_ADVFH      0x2E
#define OV7670_YAVE       0x2F
#define OV7670_HSYST      0x30
#define OV7670_HSYEN      0x31
#define OV7670_HREF       0x32
#define OV7670_CHLF       0x33
#define OV7670_ARBLM      0x34
//#define OV7670_RSVD       0x35
//#define OV7670_RSVD       0x36
#define OV7670_ADC        0x37
#define OV7670_ACOM       0x38
#define OV7670_OFON       0x39
#define OV7670_TSLB       0x3A
#define OV7670_COM11      0x3B
#define OV7670_COM12      0x3C
#define OV7670_COM13      0x3D
#define OV7670_COM14      0x3E
#define OV7670_EDGE       0x3F
#define OV7670_COM15      0x40
#define OV7670_COM16      0x41
#define OV7670_COM17      0x42
#define OV7670_AWBC1      0x43
#define OV7670_AWBC2      0x44
#define OV7670_AWBC3      0x45
#define OV7670_AWBC4      0x46
#define OV7670_AWBC5      0x47
#define OV7670_AWBC6      0x48
//#define OV7670_RSVD       0x49
//#define OV7670_RSVD       0x4A
#define OV7670_REG4B      0x4B
#define OV7670_DNSTH      0x4C
//#define OV7670_RSVD       0x4D
//#define OV7670_RSVD       0x4E
#define OV7670_MTX1       0x4F
#define OV7670_MTX2       0x50
#define OV7670_MTX3       0x51
#define OV7670_MTX4       0x52
#define OV7670_MTX5       0x53
#define OV7670_MTX6       0x54
#define OV7670_BRTN       0x55
#define OV7670_CONTRAS    0x56
#define OV7670_CONTRASCENTER    0x57
#define OV7670_MTXS       0x58
//#define OV7670_RSVD       0x59
//#define OV7670_RSVD       0x5A
//#define OV7670_RSVD       0x5B
//#define OV7670_RSVD       0x5C
//#define OV7670_RSVD       0x5D
//#define OV7670_RSVD       0x5E
//#define OV7670_RSVD       0x5F
//#define OV7670_RSVD       0x60
//#define OV7670_RSVD       0x61
#define OV7670_LCC1       0x62
#define OV7670_LCC2       0x63
#define OV7670_LCC3       0x64
#define OV7670_LCC4       0x65
#define OV7670_LCC5       0x66
#define OV7670_MANU       0x67
#define OV7670_MANV       0x68
#define OV7670_GFIX       0x69
#define OV7670_GGAIN      0x6A
#define OV7670_DBLV       0x6B
#define OV7670_AWBCTR3    0x6C
#define OV7670_AWBCTR2    0x6D
#define OV7670_AWBCTR1    0x6E
#define OV7670_AWBCTR0    0x6F
#define OV7670_SCALING_XSC      0x70
#define OV7670_SCALING_YSC      0x71
#define OV7670_SCALING_DCWCTR   0x72
#define OV7670_SCALING_PC       0x73
#define OV7670_REG74      0x74
#define OV7670_REG75      0x75
#define OV7670_REG76      0x76
#define OV7670_REG77      0x77
//#define OV7670_RSVD       0x78
//#define OV7670_RSVD       0x79
#define OV7670_SLOP       0x7A
#define OV7670_GAM1       0x7B
#define OV7670_GAM2       0x7C
#define OV7670_GAM3       0x7D
#define OV7670_GAM4       0x7E
#define OV7670_GAM5       0x7F
#define OV7670_GAM6       0x80
#define OV7670_GAM7       0x81
#define OV7670_GAM8       0x82
#define OV7670_GAM9       0x83
#define OV7670_GAM10      0x84
#define OV7670_GAM11      0x85
#define OV7670_GAM12      0x86
#define OV7670_GAM13      0x87
#define OV7670_GAM14      0x88
#define OV7670_GAM15      0x89
//#define OV7670_RSVD       0x8A
//#define OV7670_RSVD       0x8B
#define OV7670_RGB444     0x8C
//#define OV7670_RSVD       0x8D
//#define OV7670_RSVD       0x8E
//#define OV7670_RSVD       0x8F
//#define OV7670_RSVD       0x90
//#define OV7670_RSVD       0x91
#define OV7670_DM_LNL     0x92
#define OV7670_DM_LNH     0x93
#define OV7670_LCC6       0x94
#define OV7670_LCC7       0x95
//#define OV7670_RSVD       0x96
//#define OV7670_RSVD       0x97
//#define OV7670_RSVD       0x98
//#define OV7670_RSVD       0x99
//#define OV7670_RSVD       0x9A
//#define OV7670_RSVD       0x9B
//#define OV7670_RSVD       0x9C
#define OV7670_BD50ST     0x9D
#define OV7670_BD60ST     0x9E
#define OV7670_HAECC1     0x9F
#define OV7670_HAECC2     0xA0
//#define OV7670_RSVD       0xA1
#define OV7670_SCALING_PCLK_DELAY       0xA2
//#define OV7670_RSVD       0xA3
#define OV7670_NT_CTRL    0xA4
#define OV7670_BD50MAX    0xA5
#define OV7670_HAECC3     0xA6
#define OV7670_HAECC4     0xA7
#define OV7670_HAECC5     0xA8
#define OV7670_HAECC6     0xA9
#define OV7670_HAECC7     0xAA
#define OV7670_BD60MAX    0xAB
#define OV7670_STR_OPT    0xAC
#define OV7670_STR_R      0xAD
#define OV7670_STR_G      0xAE
#define OV7670_STR_B      0xAF
//#define OV7670_RSVD       0xB0
#define OV7670_ABLC1      0xB1
//#define OV7670_RSVD       0xB2
#define OV7670_THL_DLT    0xB3
//#define OV7670_RSVD       0xB4
#define OV7670_THL_DLT_AREA    0xB5
//#define OV7670_RSVD       0xB6
//#define OV7670_RSVD       0xB7
//#define OV7670_RSVD       0xB8
//#define OV7670_RSVD       0xBC
//#define OV7670_RSVD       0xBD
#define OV7670_AD_CHB     0xBE
#define OV7670_AD_CHR     0xBF
#define OV7670_AD_CHGb    0xC0
#define OV7670_AD_CHGr    0xC1
//#define OV7670_RSVD       0xC2
//#define OV7670_RSVD       0xC3
//#define OV7670_RSVD       0xC4
//#define OV7670_RSVD       0xC5
//#define OV7670_RSVD       0xC6
//#define OV7670_RSVD       0xC7
//#define OV7670_RSVD       0xC8
#define OV7670_SATCTR     0xC9

/* Registers bit definition */
/* COM1 Register */
#define CCIR656_FORMAT  0x40
#define HREF_SKIP_0     0x00
#define HREF_SKIP_1     0x04
#define HREF_SKIP_3     0x08

/* COM2 Register */
#define SOFT_SLEEP_MODE  0x10
#define ODCAP_1x         0x00
#define ODCAP_2x         0x01
#define ODCAP_3x         0x02
#define ODCAP_4x         0x03

/* COM3 Register */
#define COLOR_BAR_OUTPUT         0x80
#define OUTPUT_MSB_LAS_SWAP      0x40
#define PIN_REMAP_RESETB_EXPST   0x08
#define RGB565_FORMAT            0x00
#define RGB_OUTPUT_AVERAGE       0x04
#define SINGLE_FRAME             0x01

/* COM5 Register */
#define SLAM_MODE_ENABLE      0x40
#define EXPOSURE_NORMAL_MODE  0x01

/* COM7 Register */
#define SCCB_REG_RESET                       0x80
#define FORMAT_CTRL_15fpsVGA                 0x00
#define FORMAT_CTRL_30fpsVGA_NoVArioPixel    0x50
#define FORMAT_CTRL_30fpsVGA_VArioPixel      0x60
#define OUTPUT_FORMAT_RAWRGB                 0x00
#define OUTPUT_FORMAT_RAWRGB_DATA            0x00
#define OUTPUT_FORMAT_RAWRGB_INTERP          0x01
#define OUTPUT_FORMAT_YUV                    0x02
#define OUTPUT_FORMAT_RGB                    0x03

/* COM9 Register */
#define GAIN_2x         0x00
#define GAIN_4x         0x10
#define GAIN_8x         0x20
#define GAIN_16x        0x30
#define GAIN_32x        0x40
#define GAIN_64x        0x50
#define GAIN_128x       0x60
#define DROP_VSYNC      0x04
#define DROP_HREF       0x02

/* COM10 Register */
#define RESETb_REMAP_SLHS    0x80
#define HREF_CHANGE_HSYNC    0x40
#define PCLK_ON              0x00
#define PCLK_OFF             0x20
#define PCLK_POLARITY_REV    0x10
#define HREF_POLARITY_REV    0x08
#define RESET_ENDPOINT       0x04
#define VSYNC_NEG            0x02
#define HSYNC_NEG            0x01

/* TSLB Register */
#define PCLK_DELAY_0         0x00
#define PCLK_DELAY_2         0x40
#define PCLK_DELAY_4         0x80
#define PCLK_DELAY_6         0xC0
#define OUTPUT_BITWISE_REV   0x20
#define UV_NORMAL            0x00
#define UV_FIXED             0x10
#define YUV_SEQ_YUYV         0x00
#define YUV_SEQ_YVYU         0x02
#define YUV_SEQ_VYUY         0x04
#define YUV_SEQ_UYVY         0x06
#define BANDING_FREQ_50      0x02

#define RGB_NORMAL   0x00
#define RGB_565      0x10
#define RGB_555      0x30
#endif

/******************************************************************************
 *                           GLOBAL DATA TYPES                                *
 ******************************************************************************/
typedef enum
{
    OV7670_DRAWLINE_CBK,
    OV7670_DRAWFRAME_CBK,
} OV7670_CB_t;

typedef void (*OV7670_FncPtr_t)(void);


/******************************************************************************
 *                      GLOBAL FUNCTIONS PROTOTYPES                           *
 ******************************************************************************/

extern void OV7670_Init(DCMI_HandleTypeDef *hdcmi, I2C_HandleTypeDef *hi2c,
        TIM_HandleTypeDef* htim, uint32_t tim_ch);
extern void OV7670_RegisterCallback(OV7670_CB_t cb_type, OV7670_FncPtr_t fnc_ptr);
extern uint8_t OV7670_isDriverBusy(void);
extern void OV7670_Start(void);
extern void OV7670_Stop(void);

/******************************************************************************
 *                  HAL callbacks for DCMI_IRQHandler                         *
 ******************************************************************************/

#if (OV7670_STREAM_MODE == OV7670_SRTEAM_MODE_BY_FRAME)
extern void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi);
extern void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi);
#else  /* (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE) */
extern void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi);
#endif /* (OV7670_STREAM_MODE == OV7670_STREAM_MODE_BY_LINE) */

HAL_StatusTypeDef ov7670_write(uint8_t regAddr, uint8_t data);
HAL_StatusTypeDef ov7670_read(uint8_t regAddr, uint8_t *data);

#endif /* OV7670_H_ */
