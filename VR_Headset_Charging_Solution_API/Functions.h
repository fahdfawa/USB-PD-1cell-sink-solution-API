

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "Registers.h"
#include "Fields.h"
#include <stdint.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "gpio.h"
#include "led.h"
#include "i2c.h"

extern volatile int Charg_In; //Don't initialize the variable if you are using extern.

	//Creating Instance
extern mxc_gpio_cfg_t gpio_polling;
extern mxc_gpio_cfg_t gpio_interrupt;

extern mxc_gpio_cfg_t gpio_interrupt_status;
extern mxc_gpio_cfg_t gpio_interrupt_status_2;

extern mxc_gpio_cfg_t gpio_led_1;
extern mxc_gpio_cfg_t gpio_led_2;
extern mxc_gpio_cfg_t gpio_led_3;
extern mxc_gpio_cfg_t gpio_led_4;





//Function Declaration



typedef struct{

	unsigned int Req_PDO_Pos : 3;
	unsigned int Reserved    : 5;

}Src_Cap_Req;

typedef union{

	Src_Cap_Req src_cap_req;
	uint8_t Src_Cap_Req_combined;

}Src_Cap_Req_union;

typedef struct{

  uint8_t Command;
  uint8_t Conf;
  uint8_t PDO1_1;
  uint8_t PDO1_2;
  uint8_t PDO1_3;
  uint8_t PDO1_4;
  uint8_t PDO2_1;
  uint8_t PDO2_2;
  uint8_t PDO2_3;
  uint8_t PDO2_4;
  uint8_t PDO3_1;
  uint8_t PDO3_2;
  uint8_t PDO3_3;
  uint8_t PDO3_4;
  uint8_t PDO4_1;
  uint8_t PDO4_2;
  uint8_t PDO4_3;
  uint8_t PDO4_4;
  uint8_t PDO5_1;
  uint8_t PDO5_2;
  uint8_t PDO5_3;
  uint8_t PDO5_4;
  uint8_t PDO6_1;
  uint8_t PDO6_2;
  uint8_t PDO6_3;
  uint8_t PDO6_4;
  uint8_t PDO7_1;
  uint8_t PDO7_2;
  uint8_t PDO7_3;
  uint8_t PDO7_4;
  uint8_t PDO8_1;
  uint8_t PDO8_2;
  uint8_t PDO8_3;
  uint8_t PDO8_4;


}Current_Src_Cap;


typedef union{

	Current_Src_Cap cur_src_cap;
	uint8_t Current_Src_Cap_combined[40];

}Current_Src_Cap_union;

typedef struct{

  uint8_t Command;
  uint8_t No_of_PDOs;
  uint8_t PDO1_1;
  uint8_t PDO1_2;
  uint8_t PDO1_3;
  uint8_t PDO1_4;
  uint8_t PDO2_1;
  uint8_t PDO2_2;
  uint8_t PDO2_3;
  uint8_t PDO2_4;
  uint8_t PDO3_1;
  uint8_t PDO3_2;
  uint8_t PDO3_3;
  uint8_t PDO3_4;
  uint8_t PDO4_1;
  uint8_t PDO4_2;
  uint8_t PDO4_3;
  uint8_t PDO4_4;
  uint8_t PDO5_1;
  uint8_t PDO5_2;
  uint8_t PDO5_3;
  uint8_t PDO5_4;
  uint8_t PDO6_1;
  uint8_t PDO6_2;
  uint8_t PDO6_3;
  uint8_t PDO6_4;

}SNK_PDO;


typedef union{

	SNK_PDO snk_pdo;
	uint8_t SNK_PDO_Combined[40];

}SNK_PDO_union;


typedef struct
{

	unsigned int CHGDetEn        : 1;
	unsigned int CHGDetMan       : 1;
	unsigned int                 : 1;
	unsigned int Nikon_Detection : 1;
	unsigned int                 : 4;
	unsigned int DCDCpl          : 1;

}BC_CTRL1;

typedef union{

	BC_CTRL1 bc_ctrl1;
	uint8_t BC_CTRL1_combined;

}BC_CTRL_union;


typedef struct{

	unsigned int AttachedHoldM : 1;
	unsigned int ChgTypM       : 1;
	unsigned int StopModeM     : 1;
	unsigned int DCDTmoM       : 1;
	unsigned int VbADCM        : 1;
	unsigned int VBUSDetM      : 1;
	unsigned int SYSMsgM       : 1;
	unsigned int APCmdResM     : 1;

}UIC_Int_Msk;

typedef union{

	UIC_Int_Msk uic_int_mask;
	uint8_t UIC_Int_Msk_combined;

}UIC_Int_Msk_union;

typedef struct{

	unsigned int AttachedHoldI : 1;
	unsigned int ChgTypI       : 1;
	unsigned int StopModeI     : 1;
	unsigned int DCDTmoI       : 1;
	unsigned int VbADCI        : 1;
	unsigned int VBUSDetI      : 1;
	unsigned int SYSMsgI       : 1;
	unsigned int APCmdResI     : 1;

}UIC_Int;

typedef union{

	UIC_Int uic_int;
	uint8_t UIC_Int_combined;

}UIC_Int_union;

void Register_Read(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t* Read_value);
void Register_Write(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t Write_value0_LSB);
void Register_Multi_Read(mxc_i2c_regs_t* i2c_master, uint8_t start_reg, uint8_t num_regs, uint8_t* Read_values);
void Multi_Register_4byte_Write(mxc_i2c_regs_t* i2c_master, uint8_t start_reg, uint32_t Write_Value_32_bit);
void BC_CTRL1_Write(uint8_t DCDCpt, State Nikon_det, State CHGDetMan, State CHGDetEn, BC_CTRL_union *comb_bc_ctrl1 );
void BC_CTRL1_Read(BC_CTRL_union *comb_bc_ctrl1, uint8_t* BC_CTRL1_value);
void Source_Cap_Req(PDO_position Position,  Src_Cap_Req_union *comb_src_cap_req);
void Current_Source_Cap(Current_Src_Cap_union *comb_current_src_cap, uint8_t *No_of_PDOs, uint8_t *Selected_Pos, uint32_t* SRC_PDO1, uint32_t* SRC_PDO2, uint32_t* SRC_PDO3, uint32_t* SRC_PDO4, uint32_t* SRC_PDO5, uint32_t* SRC_PDO6, uint32_t* SRC_PDO7, uint32_t* SRC_PDO8);
void Set_Sink_PDOs(Memory_Write mry_write, PDO_position No_of_PDOs, uint32_t Snk_PDO1, uint32_t Snk_PDO2, uint32_t Snk_PDO3, uint32_t Snk_PDO4, uint32_t Snk_PDO5, uint32_t Snk_PDO6);
void Sink_PDO_Req(SNK_PDO_union *comb_snk_PDO_req, uint8_t* No_of_PDOs, uint32_t* SNK_PDO1, uint32_t* SNK_PDO2, uint32_t* SNK_PDO3, uint32_t* SNK_PDO4, uint32_t* SNK_PDO5, uint32_t* SNK_PDO6);
void Set_Interrupt_Mask();
void GPIO_Initialisation();
void Port_Detection_Status_Voltage_current_Read(uint8_t* Voltage_Read, uint8_t* Current_Read, uint8_t* PD_Status0);
void VBUS_ADC_Voltage_print(uint8_t* Voltage_int);
void Register_Write_Charger(uint8_t reg_addr, uint8_t Write_data, uint8_t Len);
void Register_Read_Charger(uint8_t reg_addr, uint8_t *Read_data, uint8_t Len);
void MAX77986_Forward_Charging_ON(uint32_t Term_Voltage_mV, uint16_t Curr_input_limit_mA, uint16_t Fast_chg_curr_mA);
void Enable_Reverse_OTG(uint16_t Reverse_Voltage_in_mV , uint16_t Reverse_Current_lim_in_mA);
void Forward_Chg_OFF_Reverse_OTG_OFF_Boost_on(uint16_t Reverse_Voltage_in_mV , uint16_t Reverse_Current_lim_in_mA);
void MAX77986_ALL_MODES_OFF();
void Enable_PPS_Mode(uint8_t Enable, uint16_t Default_Voltage, uint16_t default_oper_curr);
void MAX77958_Customer_Configuration(TrySNK_Mode mode, Power_Role role, Memory_Write write_enable, Moisture_Det moist_enable, uint16_t SRC_Voltage_in_mV, uint16_t SRC_Current_in_mV);
void Set_PPS_Voltage_Current(uint8_t PDO_Pos, uint16_t PPS_Volt, uint16_t PPS_Curr);
void Register_Read_FG(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t* Read_value);
void Register_Write_FG(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t Write_value0_LSB, uint8_t Write_value0_MSB);
void SOC_Indicator();
void SOC_Indicator_Blink_Fn();
void Mask_Interrupts();
void Read_and_Clear_Interrupts();
void Charger_Mode_Selection();
uint16_t MAX17262_EZ_Initialization(uint16_t Design_Cap_in_mAh, uint16_t IchgTerm_in_mA, uint16_t VEmpty_in_mV, uint16_t Vrecovery_in_mV, uint16_t Charging_Voltage_in_mV);
void Dynamic_Voltage_Neg_CV_Charg();

////Instance declaration











#endif /* FUNCTIONS_H_ */
