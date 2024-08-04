/*
 * MAX77958_Registers.h
 *
 *  Created on: Sep 28, 2023
 *      Author: Fahad Ahammad
 */

#ifndef REGISTERS_H_
#define REGISTERS_H_

///I2C Defines///

#define I2C_MASTER MXC_I2C0  //P0.8 & P0.9

///GPIO Pins///

#define BUTTON_PORT MXC_GPIO0
#define BUTTON_PIN MXC_GPIO_PIN_12

#define ON_BOARD_LED_PORT MXC_GPIO0
#define ON_BOARD_LED_PIN MXC_GPIO_PIN_13

#define LED1_PORT MXC_GPIO0
#define LED1_PIN MXC_GPIO_PIN_5

#define LED2_PORT MXC_GPIO0
#define LED2_PIN MXC_GPIO_PIN_6

#define LED3_PORT MXC_GPIO0
#define LED3_PIN MXC_GPIO_PIN_7

#define LED4_PORT MXC_GPIO0
#define LED4_PIN MXC_GPIO_PIN_4

#define MAX77958_INT_PORT MXC_GPIO0
#define MAX77958_INT_PIN MXC_GPIO_PIN_11

#define DUMMY_TOGGLE_PORT MXC_GPIO0
#define DUMMY_TOGGLE_PIN MXC_GPIO_PIN_10

///MAX77958 BASIC REGISTERS///

#define  DEVICE_ID     0x00
#define  DEVICE_REV    0x01
#define  FW_REV        0x02
#define  FW_SUB_VER    0x03
#define  UIC_INT       0x04
#define  CC_INT        0x05
#define  PD_INT        0x06
#define  ACTION_INT    0x07
#define  USBC_STATUS1  0x08
#define  USBC_STATUS2  0x09
#define  BC_STATUS     0x0A
#define  DP_STATUS     0x0B
#define  CC_STATUS0    0x0C
#define  CC_STATUS1    0x0D
#define  PD_STATUS0    0x0E
#define  PD_STATUS1    0x0F
#define  UIC_INT_M     0x10
#define  CC_INT_M      0x11
#define  PD_INT_M      0x12
#define  ACTION_INT_M  0x13

///MAX77958 COMMAND REGISTERS///

#define BC_CTRL1_CONFIG_READ     0x01
#define BC_CTRL1_CONFIG_WRITE    0x02
#define BC_CTRL2_CONFIG_READ     0x03
#define BC_CTRL2_CONFIG_WRITE    0x04
#define CONTROL1_READ            0x05
#define CONTROL_WRITE            0x06
#define CC_CONTROL1_READ         0x0B
#define CC_CONTROL1_WRITE        0x0C
#define CC_CONTROL4_READ         0x11
#define CC_CONTROL4_WRITE        0x12
#define GPIO_CONTROL_READ        0x23
#define GPIO_CONTROL_WRITE       0x24
#define GPIO0_GPIO1_ADC_READ     0x27
#define GET_SNK_CAP              0x2F
#define CUR_SEL_SRC_CAP          0x30
#define GET_SRC_CAP              0x31
#define SRC_CAP_REQ              0x32
#define SET_SRC_CAP              0x33
#define SEND_GET_REQ             0x34
#define READ_GET_REQ_RESP        0x35
#define SEND_GET_RESP            0x36
#define SWAP_REQ                 0x37
#define SWAP_REQ_RESPONSE        0x38
#define APDO_SRCCAP_REQUEST      0x3A
#define SET_PPS                  0x3C
#define SNK_PDO_REQUEST_READ     0x3E
#define SNK_PDO_SET              0x3F
#define GETPDMSG                 0x4A
#define CUSTOM_CONFIG_READ       0x55
#define CUSTOM_CONFIG_WRITE      0x56
#define GPIO7_GPIO8_INT_SET_REQ  0x64
#define MASTER_I2C_READ          0x85
#define MASTER_I2C_WRITE         0x86


///////MAX77986 Registers//////

#define SLAVE_WRITE_ADDRESS 0xD6
#define SLAVE_READ_ADDRESS  0xD7

#define CHG_CNFG_00  0x16
#define CHG_DTLS_01  0x14
#define CHG_CNFG_02  0x18
#define CHG_CNFG_04  0x1A
#define CHG_CNFG_06  0x1C
#define CHG_CNFG_08  0x1E
#define CHG_CNFG_09  0x1F
#define CHG_CNFG_10  0x20
#define CHG_CNFG_11  0x21

/////MAX17262 Registers/////

#define FG_SLAVE_WR_ADDR   0x6C
#define FG_SLAVE_RD_ADDR   0x6D

#define STATUS    0x00
#define VCELL     0x09
#define CURRENT   0x0A
#define REPSOC    0x06

#define DESIGN_CAP 0x18
#define ICHRTERM   0x1E
#define VEMPTY     0x3A
#define FSTAT      0x3D

#define MODEL_CFG  0xDB
#define DEVICE_NAME 0x11





#endif /* REGISTERS_H_ */
