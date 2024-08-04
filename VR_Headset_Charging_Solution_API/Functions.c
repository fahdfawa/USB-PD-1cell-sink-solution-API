/*
 * FUNCTIONs.c
 *
 *  Created on: Oct 12, 2023
 *      Author: AN
 */


#include "FUNCTIONs.h"
#include "REGISTERs.h"
#include "FIELDs.h"
#include "i2c.h"
#include "gpio.h"

volatile int Charg_In = 0;
int flag = FALSE;

mxc_i2c_req_t reqMaster;

//Creating Instance
mxc_gpio_cfg_t gpio_polling;
mxc_gpio_cfg_t gpio_interrupt;

mxc_gpio_cfg_t gpio_interrupt_status;
mxc_gpio_cfg_t gpio_interrupt_status_2;

mxc_gpio_cfg_t gpio_led_1;
mxc_gpio_cfg_t gpio_led_2;
mxc_gpio_cfg_t gpio_led_3;
mxc_gpio_cfg_t gpio_led_4;

//volatile int Charg_In = 0;


uint8_t USB_STATUS_1_Read[1];
uint8_t CC_Istat[1];
uint8_t Voltage_Read;
uint8_t Current_Read;

uint8_t UIC_Interrupt[1];
uint8_t CC_Interrupt[1];
uint8_t PD_Interrupt[1];
uint8_t Action_Interrupt[1];

uint8_t CC_STATUS0_return[1];


const float SOC_factor = 1.0/256.0;
const float Rsense = 0.01;
const float Voltage_factor = 78.125e-6;
const float Current_factor = (1.5625e-6)/Rsense;

uint8_t SOC_returned[2];
uint8_t Vcell[2];
uint8_t Current[2];




void Register_Read(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t* Read_value)
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x25;
	reqMaster.tx_buf = &reg;                                     //tx_buf is a pointer
	reqMaster.tx_len = sizeof(reg);
	reqMaster.rx_buf = Read_value;                               //
	reqMaster.rx_len = 1;                                        //rx_len !=0 fir read operation
	MXC_I2C_MasterTransaction(&reqMaster);

}

void Register_Write(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t Write_value0_LSB) //
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x25;
	uint8_t buf[2] = {reg, Write_value0_LSB}; // is write data is more than 16bit, add Write_value1_LSB, Write_value1_MSB and so on...
	reqMaster.tx_buf = buf;
	reqMaster.tx_len = sizeof(buf);
	reqMaster.rx_len = 0;                                        //rx_len = 0 for write operation
	MXC_I2C_MasterTransaction(&reqMaster);
}


void Register_Multi_Read(mxc_i2c_regs_t* i2c_master, uint8_t start_reg, uint8_t num_regs, uint8_t* Read_values)
{

    for (uint8_t i = 0; i < num_regs; i++) {
        Register_Read(i2c_master, start_reg + i, &Read_values[i]);
    }
}

void Multi_Register_4byte_Write(mxc_i2c_regs_t* i2c_master, uint8_t start_reg, uint32_t Write_Value_32_bit){
	    uint8_t write_buff[4];

	    write_buff[0] = (uint8_t)(0xFF & Write_Value_32_bit);        // Least significant byte (LSB)
	    write_buff[1] = (uint8_t)(0xFF & Write_Value_32_bit >> 8);
	    write_buff[2] = (uint8_t)(0xFF & Write_Value_32_bit >> 16);
	    write_buff[3] = (uint8_t)(0xFF & Write_Value_32_bit >> 24);

	for (uint8_t i = 0; i < 4; i++) {
	     Register_Write(i2c_master, start_reg + i, write_buff[i]);
	}
}


void Register_Write_Charger(uint8_t reg_addr, uint8_t Write_data, uint8_t Len)
{
    Register_Write(I2C_MASTER,COMMAND_WRITE_ADDRESS,MASTER_I2C_WRITE);
    Register_Write(I2C_MASTER,COMMAND_WRITE_DATA_0,SLAVE_WRITE_ADDRESS);
    Register_Write(I2C_MASTER,COMMAND_WRITE_DATA_1,reg_addr);
    Register_Write(I2C_MASTER,COMMAND_WRITE_DATA_2,Len);
    Register_Write(I2C_MASTER,COMMAND_WRITE_DATA_3,Write_data);
    Register_Write(I2C_MASTER,COMMAND_END_ADDRESS,0x00);

}

void Register_Read_Charger(uint8_t reg_addr, uint8_t *Read_data, uint8_t Len)
{
	Register_Write(I2C_MASTER,COMMAND_WRITE_ADDRESS,MASTER_I2C_READ);
	Register_Write(I2C_MASTER,COMMAND_WRITE_DATA_0,SLAVE_WRITE_ADDRESS);
	Register_Write(I2C_MASTER,COMMAND_WRITE_DATA_1,reg_addr);
	Register_Write(I2C_MASTER,COMMAND_WRITE_DATA_2,Len);
	Register_Write(I2C_MASTER,COMMAND_END_ADDRESS,0x00);
    MXC_Delay(1000000);
	Register_Read(I2C_MASTER, COMMAND_READ_DATA_3, Read_data);

}




void BC_CTRL1_Write(uint8_t DCDCpt, State Nikon_det, State CHGDetMan, State CHGDetEn, BC_CTRL_union *comb_bc_ctrl1 ){

	comb_bc_ctrl1->bc_ctrl1.CHGDetEn  = CHGDetEn;
	comb_bc_ctrl1->bc_ctrl1.CHGDetMan = CHGDetMan;
	comb_bc_ctrl1->bc_ctrl1.Nikon_Detection = Nikon_det;
	comb_bc_ctrl1->bc_ctrl1.DCDCpl   = DCDCpt;

	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, BC_CTRL1_CONFIG_WRITE);
	Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0, comb_bc_ctrl1->BC_CTRL1_combined);
	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
}


void BC_CTRL1_Read(BC_CTRL_union *comb_bc_ctrl1, uint8_t* BC_CTRL1_value){

	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, BC_CTRL1_CONFIG_WRITE);
	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);

	Register_Read(I2C_MASTER, COMMAND_READ_ADDRESS ,&comb_bc_ctrl1->BC_CTRL1_combined);
	*BC_CTRL1_value =  comb_bc_ctrl1->BC_CTRL1_combined;

}

void Source_Cap_Req(PDO_position Position,  Src_Cap_Req_union *comb_src_cap_req){

	 comb_src_cap_req->src_cap_req.Req_PDO_Pos = Position;
	 Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, SRC_CAP_REQ);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0, comb_src_cap_req->Src_Cap_Req_combined);
	 Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
}


void Current_Source_Cap(Current_Src_Cap_union *comb_current_src_cap, uint8_t *No_of_PDOs, uint8_t *Selected_Pos, uint32_t* SRC_PDO1, uint32_t* SRC_PDO2, uint32_t* SRC_PDO3, uint32_t* SRC_PDO4, uint32_t* SRC_PDO5, uint32_t* SRC_PDO6, uint32_t* SRC_PDO7, uint32_t* SRC_PDO8){

	uint8_t NO_OF_READ_ADDRESS = 36;
	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, CUR_SEL_SRC_CAP);
	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
	Register_Multi_Read(I2C_MASTER, COMMAND_READ_ADDRESS, NO_OF_READ_ADDRESS, comb_current_src_cap->Current_Src_Cap_combined);
	*No_of_PDOs = comb_current_src_cap->cur_src_cap.Conf & 0b00000111;
	*Selected_Pos = (comb_current_src_cap->cur_src_cap.Conf >> 3) & 0b00000111;
    *SRC_PDO1 = comb_current_src_cap->cur_src_cap.PDO1_1 | comb_current_src_cap->cur_src_cap.PDO1_2 << 8 | comb_current_src_cap->cur_src_cap.PDO1_3 << 16 | comb_current_src_cap->cur_src_cap.PDO1_4 << 24 ;
    *SRC_PDO2 = comb_current_src_cap->cur_src_cap.PDO2_1 | comb_current_src_cap->cur_src_cap.PDO2_2 << 8 | comb_current_src_cap->cur_src_cap.PDO2_3 << 16 | comb_current_src_cap->cur_src_cap.PDO2_4 << 24 ;
    *SRC_PDO3 = comb_current_src_cap->cur_src_cap.PDO3_1 | comb_current_src_cap->cur_src_cap.PDO3_2 << 8 | comb_current_src_cap->cur_src_cap.PDO3_3 << 16 | comb_current_src_cap->cur_src_cap.PDO3_4 << 24 ;
    *SRC_PDO4 = comb_current_src_cap->cur_src_cap.PDO4_1 | comb_current_src_cap->cur_src_cap.PDO4_2 << 8 | comb_current_src_cap->cur_src_cap.PDO4_3 << 16 | comb_current_src_cap->cur_src_cap.PDO4_4 << 24 ;
    *SRC_PDO5 = comb_current_src_cap->cur_src_cap.PDO5_1 | comb_current_src_cap->cur_src_cap.PDO5_2 << 8 | comb_current_src_cap->cur_src_cap.PDO5_3 << 16 | comb_current_src_cap->cur_src_cap.PDO5_4 << 24 ;
	*SRC_PDO6 = comb_current_src_cap->cur_src_cap.PDO6_1 | comb_current_src_cap->cur_src_cap.PDO6_2 << 8 | comb_current_src_cap->cur_src_cap.PDO6_3 << 16 | comb_current_src_cap->cur_src_cap.PDO6_4 << 24 ;
	*SRC_PDO7 = comb_current_src_cap->cur_src_cap.PDO7_1 | comb_current_src_cap->cur_src_cap.PDO7_2 << 8 | comb_current_src_cap->cur_src_cap.PDO7_3 << 16 | comb_current_src_cap->cur_src_cap.PDO7_4 << 24 ;
	*SRC_PDO8 = comb_current_src_cap->cur_src_cap.PDO8_1 | comb_current_src_cap->cur_src_cap.PDO8_2 << 8 | comb_current_src_cap->cur_src_cap.PDO8_3 << 16 | comb_current_src_cap->cur_src_cap.PDO8_4 << 24 ;

}

void Set_Sink_PDOs(Memory_Write mry_write, PDO_position No_of_PDOs, uint32_t Snk_PDO1, uint32_t Snk_PDO2, uint32_t Snk_PDO3, uint32_t Snk_PDO4, uint32_t Snk_PDO5, uint32_t Snk_PDO6){

	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, SNK_PDO_SET);
	uint8_t command_write_data_0 = 0b10000111 & ((mry_write << 7) | No_of_PDOs);
	Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0,command_write_data_0) ;
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_1,  Snk_PDO1);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_5,  Snk_PDO2);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_9,  Snk_PDO3);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_13, Snk_PDO4);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_17, Snk_PDO5);
	Multi_Register_4byte_Write(I2C_MASTER, COMMAND_WRITE_DATA_21, Snk_PDO6);

	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);

}

void Sink_PDO_Req(SNK_PDO_union *comb_snk_PDO_req, uint8_t* No_of_PDOs, uint32_t* SNK_PDO1, uint32_t* SNK_PDO2, uint32_t* SNK_PDO3, uint32_t* SNK_PDO4, uint32_t* SNK_PDO5, uint32_t* SNK_PDO6){

	uint8_t NO_OF_READ_ADDRESS = 36;
	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, SNK_PDO_REQUEST_READ);
	Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
	Register_Multi_Read(I2C_MASTER, COMMAND_READ_ADDRESS, NO_OF_READ_ADDRESS, comb_snk_PDO_req->SNK_PDO_Combined);
    *No_of_PDOs = comb_snk_PDO_req->snk_pdo.No_of_PDOs;
	*SNK_PDO1 = comb_snk_PDO_req->snk_pdo.PDO1_1 | comb_snk_PDO_req->snk_pdo.PDO1_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO1_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO1_4 << 24 ;
	*SNK_PDO2 = comb_snk_PDO_req->snk_pdo.PDO2_1 | comb_snk_PDO_req->snk_pdo.PDO2_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO2_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO2_4 << 24 ;
	*SNK_PDO3 = comb_snk_PDO_req->snk_pdo.PDO3_1 | comb_snk_PDO_req->snk_pdo.PDO3_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO3_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO3_4 << 24 ;
	*SNK_PDO4 = comb_snk_PDO_req->snk_pdo.PDO4_1 | comb_snk_PDO_req->snk_pdo.PDO4_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO4_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO4_4 << 24 ;
	*SNK_PDO5 = comb_snk_PDO_req->snk_pdo.PDO5_1 | comb_snk_PDO_req->snk_pdo.PDO5_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO5_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO5_4 << 24 ;
	*SNK_PDO6 = comb_snk_PDO_req->snk_pdo.PDO6_1 | comb_snk_PDO_req->snk_pdo.PDO6_2 << 8 | comb_snk_PDO_req->snk_pdo.PDO6_3 << 16 | comb_snk_PDO_req->snk_pdo.PDO6_4 << 24 ;


}

//void Set_Interrupt_Mask(){
//  Register_Write(I2C_MASTER, UIC_INT_M, 0b10000000);
//}

//void gpio_isr(void *cbdata)
//{
//	uint8_t UIC_Int_Stat[1];
//	//printf("Interrupt Status after: %x\n",UIC_Stat);
//	Register_Read(I2C_MASTER, UIC_INT, UIC_Int_Stat);
//	uint8_t UIC_Stat = UIC_Int_Stat[0];
//	printf("Interrupt Status: %x\n",UIC_Stat);
//    mxc_gpio_cfg_t *cfg = cbdata;
//
//    uint16_t Device_ID;
//    uint8_t dev_id[1];
//
//    Register_Read(I2C_MASTER, DEVICE_ID, dev_id);
//    Device_ID = 0xFF & (dev_id[0]);
//    printf("Device_ID = %x\n", Device_ID);
//
////    MXC_Delay(1000000);
//}

//Interrupt for setting the mode during plugin
void gpio_isr(void *cbdata)
{

  Charger_Mode_Selection();  //Forward or Reverse OTG mode based on CCStat

  Read_and_Clear_Interrupts();  //Clearing MAX77958 Interrupt

  MXC_Delay(100000); //Delay for eliminating false interrupt trigger during type C connection

}



void GPIO_Initialisation(){


	/* Setup interrupt status pin as an output so we can toggle it on each interrupt. */
	gpio_interrupt_status.port = ON_BOARD_LED_PORT;
	gpio_interrupt_status.mask = ON_BOARD_LED_PIN;
	gpio_interrupt_status.pad = MXC_GPIO_PAD_NONE;
	gpio_interrupt_status.func = MXC_GPIO_FUNC_OUT;
	gpio_interrupt_status.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_interrupt_status);

//Polling configured from button (pin 12) to read Fuel gauge parameters
	gpio_polling.port = BUTTON_PORT;
	gpio_polling.mask = BUTTON_PIN;
	gpio_polling.pad = MXC_GPIO_PAD_PULL_UP;
	gpio_polling.func = MXC_GPIO_FUNC_IN;
	gpio_polling.vssel = MXC_GPIO_VSSEL_VDDIOH;
	MXC_GPIO_Config(&gpio_polling);


//Interrupt configured from MAX77958 to read CC status
	gpio_interrupt.port = MAX77958_INT_PORT;
	gpio_interrupt.mask = MAX77958_INT_PIN;
	gpio_interrupt.pad = MXC_GPIO_PAD_PULL_UP;
	gpio_interrupt.func = MXC_GPIO_FUNC_IN;
	gpio_interrupt.vssel = MXC_GPIO_VSSEL_VDDIOH;
	MXC_GPIO_Config(&gpio_interrupt);
	MXC_GPIO_RegisterCallback(&gpio_interrupt, gpio_isr, &gpio_interrupt_status);
	MXC_GPIO_IntConfig(&gpio_interrupt, MXC_GPIO_INT_FALLING);
	MXC_GPIO_EnableInt(gpio_interrupt.port, gpio_interrupt.mask);
	NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MAX77958_INT_PORT)));

	//Interrupt source from AP
//	gpio_interrupt_2.port = AP_INTRPT_PORT;
//	gpio_interrupt_2.mask = AP_INTRPT_PIN;
//	gpio_interrupt_2.pad = MXC_GPIO_PAD_PULL_UP;
//	gpio_interrupt_2.func = MXC_GPIO_FUNC_IN;
//	gpio_interrupt_2.vssel = MXC_GPIO_VSSEL_VDDIOH;
//	MXC_GPIO_Config(&gpio_interrupt_2);
//	MXC_GPIO_RegisterCallback(&gpio_interrupt_2, gpio_isr_2, &gpio_interrupt_status_2);
//	MXC_GPIO_IntConfig(&gpio_interrupt_2, MXC_GPIO_INT_FALLING);
//	MXC_GPIO_EnableInt(gpio_interrupt_2.port, gpio_interrupt_2.mask);
//	NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(AP_INTRPT_PORT)));

	gpio_led_1.port = LED1_PORT;
	gpio_led_1.mask = LED1_PIN;
	gpio_led_1.pad = MXC_GPIO_PAD_NONE;
	gpio_led_1.func = MXC_GPIO_FUNC_OUT;
	gpio_led_1.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_led_1);

	gpio_led_2.port = LED2_PORT;
	gpio_led_2.mask = LED2_PIN;
	gpio_led_2.pad = MXC_GPIO_PAD_NONE;
	gpio_led_2.func = MXC_GPIO_FUNC_OUT;
	gpio_led_2.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_led_2);

	gpio_led_3.port = LED3_PORT;
	gpio_led_3.mask = LED3_PIN;
	gpio_led_3.pad = MXC_GPIO_PAD_NONE;
	gpio_led_3.func = MXC_GPIO_FUNC_OUT;
	gpio_led_3.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_led_3);

	gpio_led_4.port = LED4_PORT;
	gpio_led_4.mask = LED4_PIN;
	gpio_led_4.pad = MXC_GPIO_PAD_NONE;
	gpio_led_4.func = MXC_GPIO_FUNC_OUT;
	gpio_led_4.vssel = MXC_GPIO_VSSEL_VDDIO;
	MXC_GPIO_Config(&gpio_led_4);

}


void VBUS_ADC_Voltage_print(uint8_t* Voltage_int){
	switch(*Voltage_int)
	{
	case 0x00: printf("VBUS: 0x00 = VBUS < 3.5V\n");
	break;

	case 0x01: printf("VBUS: 0x01 = 3.5V < VBUS < 4.5V\n");
	break;

	case 0x02: printf("VBUS: 0x02 = 4.5V < VBUS < 5.5V\n");
	break;

	case 0x03: printf("VBUS: 0x03 = 5.5V < VBUS < 6.5V\n");
	break;

	case 0x04: printf("VBUS: 0x04 = 6.5V < VBUS < 7.5V\n");
	break;

	case 0x05: printf("VBUS: 0x05 = 7.5V < VBUS < 8.5V\n");
	break;

	case 0x06: printf("VBUS: 0x06 = 8.5V < VBUS < 9.5V\n");
	break;

	case 0x07: printf("VBUS: 0x07 = 9.5V < VBUS < 10.5V\n");
	break;

	case 0x08: printf("VBUS: 0x08 = 10.5V < VBUS < 11.5V\n");
	break;

	case 0x09: printf("VBUS: 0x09 = 11.5V < VBUS < 12.5V\n");
	break;

	case 0x0A: printf("VBUS: 0x0A = 12.5V < VBUS < 13.5V\n");
	break;

	case 0x0B: printf("VBUS: 0x0B = 13.5V < VBUS < 14.5V\n");
	break;

	case 0x0C: printf("VBUS: 0x0C = 14.5V < VBUS < 15.5V\n");
	break;

	case 0x0D: printf("VBUS: 0x0D = 15.5V < VBUS < 16.5V\n");
	break;

	case 0x0E: printf("VBUS: 0x0E = 16.5V < VBUS < 17.5V\n");
	break;

	case 0x0F: printf("VBUS: 0x0F = 17.5V < VBUS < 18.5V\n");
	break;

	case 0x10: printf("VBUS: 0x10 = 18.5V < VBUS < 19.5V\n");
	break;

	case 0x11: printf("VBUS: 0x11 = 19.5V < VBUS < 20.5V\n");
	break;

	case 0x12: printf("VBUS: 0x12 = 19.5V < VBUS < 20.5V\n");
	break;

	case 0x13: printf("VBUS: 0x13 = 21.5V < VBUS < 22.5V\n");
	break;

	case 0x14: printf("VBUS: 0x14 = 22.5V < VBUS < 23.5V\n");
	break;

	case 0x15: printf("VBUS: 0x15 = 23.5V < VBUS < 24.5V\n");
	break;

	case 0x16: printf("VBUS: 0x16 = 24.5V < VBUS < 25.5V \n");
	break;

	case 0x17: printf("VBUS: 0x17 = 25.5V < VBUS < 26.5V\n");
	break;

	case 0x18: printf("VBUS: 0x18 = 26.5V < VBUS < 27.5V\n");
	break;

	case 0x19: printf("VBUS: 0x19 = 27.5V < VBUS\n");
	break;

	case 0x1A: printf("VBUS: 0x1A = Reserved\n");
	break;

	default: printf("Unknown VBUS");
	break;
	}
}

void Port_current_print(uint8_t* Current_Int){
	switch(*Current_Int){

	case 0x00: printf("CC detected current limit: Not in UFP mode\n");
	break;

	case 0x01: printf("CC detected current limit: 500mA\n");
	break;

	case 0x02: printf("CC detected current limit: 1500mA\n");
	break;

	case 0x03: printf("CC detected current limit: 3000mA\n");
	break;

	default: printf("Unknown");
	break;
	}
}

void PD_Message_Print(uint8_t* PD_Mssg){
	switch(*PD_Mssg)
	{
	case 0x00: printf("PD Message: Nothing happened\n");
	break;

	case 0x01: printf("PD Message: Sink_PD_PSRdy_Received\n");
	break;

	case 0x02: printf("PD Message: Sink_PD_Error_Recovery\n");
	break;

	case 0x03: printf("PD Message: Sink_PD_SenderResponseTimer_Timeout\n");
	break;

	case 0x04: printf("PD Message: Source_PSRdy_Sent\n");
	break;

	case 0x05: printf("PD Message: Source_PD_Error_Recovery\n");
	break;

	case 0x06: printf("PD Message: Source_PD_SenderResponseTimer_Timeout\n");
	break;

	case 0x07: printf("PD Message: PD_DR_Swap_Request_Received\n");
	break;

	case 0x08: printf("PD Message: PD_PR_Swap_Request_Received\n");
	break;

	case 0x09: printf("PD Message: PD_VCONN_Swap_Request_Received\n");
	break;

	case 0x0A: printf("PD Message: Received PD Message in illegal state\n");
	break;

	case 0x0B: printf("PD Message: Sink_PD_Evaluate_State, SrcCap_Received\n");
	break;

	case 0x11: printf("PD Message: VDM Attention Message Received\n");
	break;

	case 0x12: printf("PD Message: Reject_Received\n");
	break;

	case 0x13: printf("PD Message: Not_Supported_Received\n");
	break;

	case 0x14: printf("PD_PR_Swap_SNKTOSRC_Cleanup\n");
	break;

	case 0x15: printf("PD Message: PD_PR_Swap_SRCTOSNK_Cleanup\n");
	break;

	case 0x16: printf("PD Message: HardReset_Received\n");
	break;

	case 0x17: printf("PD Message: PD_PowerSupply_VbusEnable\n");
	break;

	case 0x18: printf("PD Message: PD_PowerSupply_VbusDisable\n");
	break;

	case 0x19: printf("PD Message: HardReset_Sent\n");
	break;

	case 0x1A: printf("PD Message: PD_PR_Swap_SRCTOSWAP\n");
	break;

	case 0x1B: printf("PD Message: PD_PR_Swap_SWAPTOSNK\n");
	break;

	case 0x1C: printf("PD Message: PD_PR_Swap_SNKTOSWAP\n");
	break;

	case 0x1D: printf("PD_PR_Swap_SWAPTOSRC\n");
	break;

	case 0x20: printf("PD Message: Sink_PD_Disabled\n");
	break;

	case 0x21: printf("PD Message: Source_PD_Disabled\n");
	break;

	case 0x30: printf("PD Message: Get_Source_Capabilities_Extended_Received\n");
	break;

	case 0x31: printf("PD Message: Get_Status_Received\n");
	break;

	case 0x32: printf("PD Message: Get_Battery_Cap_Received\n");
	break;

	case 0x33: printf("PD Message: Get_Battery_Status_Received\n");
	break;

	case 0x34: printf("PD Message: Get_Manufacturer_Info_Received\n");
	break;

	case 0x35: printf("PD Message: Source_Capabilities_Extended_Received\n");
	break;

	case 0x36: printf("PD Message: Status_Received\n");
	break;

	case 0x37: printf("PD Message: Battery_Capabilities_Received\n");
	break;

	case 0x38: printf("PD Message: Battery_Status_Received\n");
	break;

	case 0x39: printf("PD Message: Manufacturer_Info_Received\n");
	break;

	case 0x3A: printf("PD Message: Security_Request_Received\n");
	break;

	case 0x3B: printf("PD Message: Security_Response_Received\n");
	break;

	case 0x3C: printf("PD Message: Firmware_Update_Request_Received\n");
	break;

	case 0x3D: printf("PD Message: Firmware_Update_Response_Received\n");
	break;

	case 0x3E: printf("PD Message: Alert_Received\n");
	break;

	case 0x40: printf("PD Message: VDM_NAK_Received\n");
	break;

	case 0x41: printf("PD Message: VDM_BUSY_Received\n");
	break;

	case 0x42: printf("PD Message: VDM_ACK_Received\n");
	break;

	case 0x43: printf("PD Message: VDM_REQ_Received\n");
	break;

	case 0x63: printf("PD Message: DiscoverMode_Received\n");
	break;

	case 0x65: printf("PD Message: PD_Status_Received\n");
	break;

	default: printf("Unknown\n");
	break;

	}
}



void Port_Detection_Status_Voltage_current_Read(uint8_t* Voltage_Read, uint8_t* Current_Read, uint8_t* PD_Status0){
	uint8_t BC_STATUS_return[1];
	Port_type Charger_type;
	uint8_t CC_Status1_ret[1];
	uint8_t PD_Status0_ret[1];
	uint8_t PD_Status1_ret[1];


	Register_Read(I2C_MASTER, BC_STATUS, BC_STATUS_return);


	Charger_type = (0b00000011 & BC_STATUS_return[0]);

	switch(Charger_type){

	case Nothing_Attached:
		 printf("Port type: Nothing Attached\n");
		 Register_Read(I2C_MASTER, USBC_STATUS1, USB_STATUS_1_Read);
		 Register_Read(I2C_MASTER, CC_STATUS0, CC_Istat);
		 *Voltage_Read = 0b00011111 & (USB_STATUS_1_Read[0] >> 3);
		 *Current_Read = 0b00000011 & (CC_Istat[0] >> 4);
		 break;

	case SDP:
		 printf("Port type: SDP\n");
		 Register_Read(I2C_MASTER, USBC_STATUS1, USB_STATUS_1_Read);
		 Register_Read(I2C_MASTER, CC_STATUS0, CC_Istat);
		 *Voltage_Read = 0b00011111 & (USB_STATUS_1_Read[0] >> 3);
		 *Current_Read = 0b00000011 & (CC_Istat[0] >> 4);
		 break;

	case CDP:
		printf("Port type: CDP\n");
		Register_Read(I2C_MASTER, USBC_STATUS1, USB_STATUS_1_Read);
		Register_Read(I2C_MASTER, CC_STATUS0, CC_Istat);
		*Voltage_Read = 0b00011111 & (USB_STATUS_1_Read[0] >> 3);
		*Current_Read = 0b00000011 & (CC_Istat[0] >> 4);
		break;

	case DCP:
		printf("Port type: DCP\n");
		Register_Read(I2C_MASTER, USBC_STATUS1, USB_STATUS_1_Read);
		Register_Read(I2C_MASTER, CC_STATUS0, CC_Istat);
		*Voltage_Read = 0b00011111 & (USB_STATUS_1_Read[0] >> 3);
		*Current_Read = 0b00000011 & (CC_Istat[0] >> 4);
        break;

	default:
		printf("Port Error\n");
		break;
	}

	VBUS_ADC_Voltage_print(Voltage_Read);
	Port_current_print(Current_Read);

	uint8_t VBUS_det = 0b00000001 & (BC_STATUS_return[0] >> 7);

	switch(VBUS_det)
	{

	case 0x0: printf("VBUS < VBDET\n");
	break;

	case 0x1: printf("VBUS > VBDET\n");
    break;

	default: printf("Unknown");
	break;
	}

	uint8_t CC_pin_state_mac = 0b00000111 & (CC_Istat[0]);

	switch(CC_pin_state_mac)
	{
	case 0x00: printf("CC_pin_machine: No connection\n");
	break;

	case 0x01: printf("CC_pin_machine: SINK\n");
    break;

	case 0x02: printf("CC_pin_machine: SOURCE\n");
	break;

	case 0x03: printf("CC_pin_machine: Audio accessory\n");
	break;

	case 0x04: printf("CC_pin_machine: DebugSrc accessory\n");
	break;

	case 0x05: printf("CC_pin_machine: Error\n");
	break;

	case 0x06: printf("CC_pin_machine: Disabled\n");
	break;

	case 0x07: printf("CC_pin_machine: DebugSnk accessory\n");
	break;

	default: printf("Unknown\n");
    break;
	}

	uint8_t CC_pin_stat = 0b00000011 & (CC_Istat[0] >> 6);

	switch(CC_pin_stat)
	{
	case 0x00: printf("No determination\n");
	break;

	case 0x01: printf("Port orientation: CC1 Active\n");
	break;

	case 0x02: printf("Port orientation: CC2 Active\n");
	break;

	case 0x03: printf("Port orientation: RFU\n");
	break;

	default: printf("Unknown\n");
	break;
	}

	Register_Read(I2C_MASTER, CC_STATUS1, CC_Status1_ret);
	uint8_t VSAFE_OV = 0b00000001 & (CC_Status1_ret[0] >> 3);

	switch(VSAFE_OV)
	{
	case 0x00: printf("VBUS < VSAFE0V\n");
	break;

	case 0x01: printf("VBUS > VSAFE0V\n");
	break;

	default: printf("Unknown\n");
	break;
	}


	Register_Read(I2C_MASTER, PD_STATUS0, PD_Status0_ret);
    *PD_Status0 = PD_Status0_ret[0];
    PD_Message_Print(PD_Status0);


    Register_Read(I2C_MASTER, PD_STATUS1, PD_Status1_ret);
    uint8_t PSRDY = 0b00000001 & (PD_Status1_ret[0] >> 4);

    switch(PSRDY)
    {
    case 0x00: printf("Nothing happened\n");
    break;

    case 0x01: printf("PSRDY received\n");
    break;

    default: printf("Unknown\n");
    break;
    }

    uint8_t Power_Role = 0b00000001 & (PD_Status1_ret[0] >> 6);

    switch(Power_Role)
    {
    case 0x00: printf("Power role: Sink\n");
    break;

    case 0x01: printf("Power role: Source\n");
    break;

    default: printf("Unknown\n");
    break;
    }

    uint8_t Data_Role = 0b00000001 & (PD_Status1_ret[0] >> 7);

    switch(Data_Role)
    {
    case 0x00: printf("Data Role: UFP\n");
    break;

    case 0x01: printf("Data Role: DFP\n");
    break;

    default: printf("Unknown\n");
    break;
    }



}


void MAX77986_Forward_Charging_ON(uint32_t Term_Voltage_mV, uint16_t Curr_input_limit_mA, uint16_t Fast_chg_curr_mA)
{

   uint8_t Term_voltage_hex = (Term_Voltage_mV - 41500)/125;
   uint8_t Curr_input_limit_hex = (Curr_input_limit_mA - 50)/50;
   uint8_t Fast_chg_curr_hex = (Fast_chg_curr_mA)/50;

   Register_Write_Charger(CHG_CNFG_06,0x0C,1);  //Unlocking Charge protection
   Register_Write_Charger(CHG_CNFG_00,0x05,1);
   Register_Write_Charger(CHG_CNFG_04,Term_voltage_hex,1);
   Register_Write_Charger(CHG_CNFG_09,Curr_input_limit_hex,1);
   Register_Write_Charger(CHG_CNFG_02, (0x7F & Fast_chg_curr_hex),1);

}

void Enable_Reverse_OTG(uint16_t Reverse_Voltage_in_mV , uint16_t Reverse_Current_lim_in_mA)
{
	 uint8_t Reverse_Voltage_hex = (Reverse_Voltage_in_mV - 5000)/100;
	 uint8_t Reverse_Current_lim_hex = (Reverse_Current_lim_in_mA - 500)/100;
	 Register_Write_Charger(CHG_CNFG_06,0x0C,1);  //Unlocking Charge protection
	 Register_Write_Charger(CHG_CNFG_00,0x0A,1);
	 Register_Write_Charger(CHG_CNFG_11, Reverse_Voltage_hex, 1);
	 Register_Write_Charger(CHG_CNFG_10, 0xFF & ((1 << 7) |Reverse_Current_lim_hex), 1);
}

void Forward_Chg_OFF_Reverse_OTG_OFF_Boost_on(uint16_t Reverse_Voltage_in_mV , uint16_t Reverse_Current_lim_in_mA)
{
	 uint8_t Reverse_Voltage_hex = (Reverse_Voltage_in_mV - 5000)/100;
	 uint8_t Reverse_Current_lim_hex = (Reverse_Current_lim_in_mA - 500)/100;
	 Register_Write_Charger(CHG_CNFG_06,0x0C,1);  //Unlocking Charge protection
	 Register_Write_Charger(CHG_CNFG_00,0x09,1);
	 Register_Write_Charger(CHG_CNFG_11, Reverse_Voltage_hex, 1);
	 Register_Write_Charger(CHG_CNFG_10, 0xFF & ((1 << 7) |Reverse_Current_lim_hex), 1);
}

void MAX77986_ALL_MODES_OFF()
{
	 Register_Write_Charger(CHG_CNFG_06,0x0C,1);  //Unlocking Charge protection
	 Register_Write_Charger(CHG_CNFG_00,0x00,1);
}



void Enable_PPS_Mode(uint8_t Enable, uint16_t Default_Voltage, uint16_t default_oper_curr)
{
	 uint8_t Voltage_def_lsb = 0xFF & (Default_Voltage/20);
	 uint8_t Voltage_def_msb = 0xFF & ((Default_Voltage/20)>>8);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, SET_PPS);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0,0x01 & Enable);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_1,Voltage_def_lsb);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_2,Voltage_def_msb);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_3, 0x7F & (default_oper_curr/50));
	 Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
}


void Set_PPS_Voltage_Current(uint8_t PDO_Pos, uint16_t PPS_Volt, uint16_t PPS_Curr)
{
	 uint8_t PPS_Volt_lsb = 0xFF & (PPS_Volt/20);
	 uint8_t PPS_Volt_msb = 0xFF & ((PPS_Volt/20)>>8);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, APDO_SRCCAP_REQUEST);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0,PDO_Pos);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_1,PPS_Volt_lsb);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_2,PPS_Volt_msb);
	 Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_3, 0x7F & (PPS_Curr/50));
	 Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
}

void MAX77958_Customer_Configuration(TrySNK_Mode mode, Power_Role role, Memory_Write write_enable, Moisture_Det moist_enable, uint16_t SRC_Voltage_in_mV, uint16_t SRC_Current_in_mV)
{
	uint8_t custom_write_data_0 = 0xFF & ((mode << 3) | (role << 4) | (write_enable << 6) | (moist_enable << 7));

	uint8_t SRC_Voltage_LSB_hex = 0xFF & SRC_Voltage_in_mV;
    uint8_t SRC_Voltage_MSB_hex = 0xFF & (SRC_Current_in_mV >> 8);
    uint8_t SRC_Current_LSB_hex = 0xFF & SRC_Current_in_mV;
    uint8_t SRC_Current_MSB_hex = 0xFF & (SRC_Current_in_mV >> 8);

	Register_Write(I2C_MASTER, COMMAND_WRITE_ADDRESS, CUSTOM_CONFIG_WRITE);
    Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_0, custom_write_data_0);
    Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_6, SRC_Voltage_LSB_hex);
    Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_7, SRC_Voltage_MSB_hex);
    Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_8, SRC_Current_LSB_hex);
    Register_Write(I2C_MASTER, COMMAND_WRITE_DATA_9, SRC_Current_MSB_hex);
    Register_Write(I2C_MASTER, COMMAND_END_ADDRESS, 0x00);
}



void Register_Read_FG(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t* Read_value)
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x36;
	reqMaster.tx_buf = &reg;                                     //tx_buf is a pointer
	reqMaster.tx_len = sizeof(reg);
	reqMaster.rx_buf = Read_value;                               //
	reqMaster.rx_len = 2;                                        //rx_len !=0 fir read operation
	MXC_I2C_MasterTransaction(&reqMaster);

}

void Register_Write_FG(mxc_i2c_regs_t* i2c_master, uint8_t reg, uint8_t Write_value0_LSB, uint8_t Write_value0_MSB) //
{
	reqMaster.i2c = i2c_master;
	reqMaster.addr = 0x36;
	uint8_t buf[3] = {reg, Write_value0_LSB, Write_value0_MSB}; // is write data is more than 16bit, add Write_value1_LSB, Write_value1_MSB and so on...
	reqMaster.tx_buf = buf;
	reqMaster.tx_len = sizeof(buf);
	reqMaster.rx_len = 0;                                        //rx_len = 0 for write operation
	MXC_I2C_MasterTransaction(&reqMaster);
}

uint16_t Twos_Comp_to_unsigned_value(int16_t twosCompValue)
{
	uint16_t unsigned_value = 0;
	if ((twosCompValue & 0x8000) != 0) {
        unsigned_value = ~twosCompValue + 1;
    }
	else if((twosCompValue & 0x8000) == 0){
		unsigned_value = twosCompValue;
	}
	return unsigned_value;
}

void SOC_Indicator()
{
  //SOC reading


    Register_Read_FG(I2C_MASTER, REPSOC, SOC_returned);
    float SOC_in_Percent = (0xFFFF & (SOC_returned[1] << 8 | SOC_returned[0]))*SOC_factor;
    printf("SOC:  %f\n",(double)SOC_in_Percent);

    Register_Read_FG(I2C_MASTER, CURRENT, Current);
	float Curr_in_A = (0xFFFF & (Twos_Comp_to_unsigned_value((Current[1] << 8) | Current[0]))) *Current_factor; //printf("V_cell = %X\n",Vcell);
	printf("Curr_in_A = %f\n", (double)Curr_in_A);

	Register_Read_FG(I2C_MASTER, VCELL, Vcell);
	float Vcell_in_volt = (0xFFFF & ((Vcell[1] << 8) | Vcell[0])) * Voltage_factor; //printf("V_cell = %X\n",Vcell);
    printf("Vcell_in_V = %f\n", (double)Vcell_in_volt);

    if (SOC_in_Percent < 21)
      {
    	  MXC_GPIO_OutSet(LED1_PORT, LED1_PIN);
		  MXC_GPIO_OutClr(LED2_PORT, LED2_PIN);
		  MXC_GPIO_OutClr(LED3_PORT, LED3_PIN);
      }
    else if (21 <= SOC_in_Percent && SOC_in_Percent < 91)
	  {
    	  MXC_GPIO_OutClr(LED1_PORT, LED1_PIN);
		  MXC_GPIO_OutSet(LED2_PORT, LED2_PIN);
		  MXC_GPIO_OutClr(LED3_PORT, LED3_PIN);
	  }

    else if (91 <= SOC_in_Percent && SOC_in_Percent <= 101)
   	  {
    	  MXC_GPIO_OutClr(LED1_PORT, LED1_PIN);
	      MXC_GPIO_OutClr(LED2_PORT, LED2_PIN);
	      MXC_GPIO_OutSet(LED3_PORT, LED3_PIN);
   	  }


  MXC_Delay(6000000); //Delay for LED blinking

	MXC_GPIO_OutClr(LED1_PORT, LED1_PIN);
    MXC_GPIO_OutClr(LED2_PORT, LED2_PIN);
    MXC_GPIO_OutClr(LED3_PORT, LED3_PIN);
}

void SOC_Indicator_Blink_Fn()
{
	Register_Read_FG(I2C_MASTER, REPSOC, SOC_returned);
	float SOC_in_Percent = (0xFFFF & (SOC_returned[1] << 8 | SOC_returned[0]))*SOC_factor;
	printf("SOC:  %f\n",(double)SOC_in_Percent);

    if (SOC_in_Percent < 21)
      {
    	  MXC_GPIO_OutSet(LED1_PORT, LED1_PIN);
		  MXC_GPIO_OutClr(LED2_PORT, LED2_PIN);
		  MXC_GPIO_OutClr(LED3_PORT, LED3_PIN);
      }
    else if (21 <= SOC_in_Percent && SOC_in_Percent < 91)
	  {
    	  MXC_GPIO_OutClr(LED1_PORT, LED1_PIN);
		  MXC_GPIO_OutSet(LED2_PORT, LED2_PIN);
		  MXC_GPIO_OutClr(LED3_PORT, LED3_PIN);
	  }

    else if (91 <= SOC_in_Percent && SOC_in_Percent <= 101)
   	  {
    	  MXC_GPIO_OutClr(LED1_PORT, LED1_PIN);
	      MXC_GPIO_OutClr(LED2_PORT, LED2_PIN);
	      MXC_GPIO_OutSet(LED3_PORT, LED3_PIN);
   	  }

    MXC_Delay(500000);  //0.5sec delay blink

	MXC_GPIO_OutClr(LED1_PORT, LED1_PIN);
    MXC_GPIO_OutClr(LED2_PORT, LED2_PIN);
    MXC_GPIO_OutClr(LED3_PORT, LED3_PIN);

    MXC_Delay(500000);  //0.5sec delay blink

}

void Mask_Interrupts()
{
	Register_Write(I2C_MASTER, UIC_INT_M, 0x84);  //AP command and Stop Mode interrupt masked
	Register_Write(I2C_MASTER, CC_INT_M, 0x00);
	Register_Write(I2C_MASTER, PD_INT_M, 0x00);
	Register_Write(I2C_MASTER, ACTION_INT_M, 0x00);

}

void Read_and_Clear_Interrupts()
{
	Register_Read(I2C_MASTER, UIC_INT, UIC_Interrupt);
	Register_Read(I2C_MASTER, CC_INT, CC_Interrupt);
	Register_Read(I2C_MASTER, PD_INT, PD_Interrupt);
	Register_Read(I2C_MASTER, ACTION_INT, Action_Interrupt);

	printf("UIC_Interrupt: %x\n",UIC_Interrupt[0]);
	printf("CC_Interrupt: %x\n",CC_Interrupt[0]);
	printf("PD_Interrupt: %x\n",PD_Interrupt[0]);
	printf("Action_Interrupt: %x\n",Action_Interrupt[0]);

}



void Charger_Mode_Selection()
{
	Register_Read(I2C_MASTER, CC_STATUS0, CC_STATUS0_return);

	if((CC_STATUS0_return[0] & 0b00000111) == 0b00000001)     //Source connected
	{
	    uint32_t Termination_Voltage = 45375; //in 4.5375V
	    uint16_t Input_curr_lim  = 3000; //in mA
	    uint16_t Fast_chg_curr = 5500;  //in mA
	    MAX77986_Forward_Charging_ON(Termination_Voltage, Input_curr_lim, Fast_chg_curr);

	    Charg_In = 1;  //go inside while loop and do led blinking
	}
	else if((CC_STATUS0_return[0] & 0b00000111) == 0b00000010)  //Sink device connected
	{
		Forward_Chg_OFF_Reverse_OTG_OFF_Boost_on(5000, 3000);
		//Enable_Reverse_OTG(5000,3000); //VBYP:  5V; Current_limit: 3A
		Charg_In = 0;  //stop LED blinking when the charger is unplugged
	}
	else if((CC_STATUS0_return[0] & 0b00000111) == 0b00000000)  //Nothing Attached
	{
		//MAX77986_ALL_MODES_OFF();  //un-comment this line and comment the next line for getting the full power output when a sink device is connected at type C receptacle
		Forward_Chg_OFF_Reverse_OTG_OFF_Boost_on(5000, 3000); //charger and Reverse OTG mode

		Charg_In = 0;  //stop LED blinking when the charger is unplugged
	}
}


uint16_t MAX17262_EZ_Initialization(uint16_t Design_Cap_in_mAh, uint16_t IchgTerm_in_mA, uint16_t VEmpty_in_mV, uint16_t Vrecovery_in_mV, uint16_t Charging_Voltage_in_mV)
{
    uint8_t Status[2];
    uint8_t Fstat[2];
    uint8_t Device_ID[2];

    Register_Read_FG(I2C_MASTER, STATUS, Status);

	if ((Status[0] & 0x02) == 0x02)
	{
	//Do Initialization
		printf("Initialization Started\n");

		// Assuming Rsense_in_mohm is a constant or defined elsewhere
		const float Rsense_in_mohm = 10.0; // Example value, replace with the actual value

		// Design_Cap_hex calculation
		uint16_t Design_Cap_hex = (Design_Cap_in_mAh * Rsense_in_mohm / 5);
		uint8_t Design_Cap_hex_LSB = 0xFF & Design_Cap_hex;
		uint8_t Design_Cap_hex_MSB = 0xFF & (Design_Cap_hex >> 8);

		Register_Write_FG(I2C_MASTER, DESIGN_CAP, Design_Cap_hex_LSB, Design_Cap_hex_MSB);

		// IchgTerm_hex calculation
		uint16_t IchgTerm_hex = (uint16_t)(IchgTerm_in_mA * Rsense_in_mohm / 1.5625);  //type casting to uint16_t required as the result can be floating point and needed to converted to integer.
		uint8_t IchgTerm_hex_LSB = 0xFF & IchgTerm_hex;
		uint8_t IchgTerm_hex_MSB = 0xFF & (IchgTerm_hex >> 8);

		Register_Write_FG(I2C_MASTER, ICHRTERM, IchgTerm_hex_LSB, IchgTerm_hex_MSB);

		// V_Recovery and V_Empty calculation
		uint8_t V_Recovery = 0x7F & (Vrecovery_in_mV / 10);
		uint16_t V_Empty = 0x1FF & (VEmpty_in_mV / 40);
		uint16_t V_empty_recovery_comb = 0xFFFF & ((V_Empty << 7) | V_Recovery);

		uint8_t VEmpty_hex_LSB = 0xFF & V_empty_recovery_comb;
		uint8_t VEmpty_hex_MSB = 0xFF & (V_empty_recovery_comb >> 8);

		Register_Write_FG(I2C_MASTER, VEMPTY, VEmpty_hex_LSB, VEmpty_hex_MSB);

		//Configuring model configuration based on the cell type and charger voltage
		if(Charging_Voltage_in_mV > 4275)
		  {
		   Register_Write_FG(I2C_MASTER, MODEL_CFG, 0x00, 0x84);  //Comment if using MAX17201
		  }
		else
		  {
		   Register_Write_FG(I2C_MASTER, MODEL_CFG, 0x00, 0x80);  //Comment if using MAX17201
		  }
		//Waiting for DNR bit to be cleared automatically
		do
		{
		  Register_Read_FG(I2C_MASTER, FSTAT, Fstat);
		  MXC_Delay(10000);
		}while((Fstat[0] & 0x01) == 0x01);  //do not continue until FSTAT.DNR ==0

		//Clearing POR bit
		do
		{//Register_Read_FG(I2C_MASTER, STATUS, Status);

			uint8_t Status_New_MSB = Status[1] & 0xFF;
			uint8_t Status_New_LSB = Status[0] & 0xFD; //Setting the POR bit to 0

			Register_Write_FG(I2C_MASTER, STATUS, Status_New_LSB, Status_New_MSB);

			MXC_Delay(20000);

			Register_Read_FG(I2C_MASTER, STATUS, Status);

		}while((Status[0] & 0x02) == 0x02); //Do clearing of POR bit until its cleared.

	}
	else
	{
	      printf("POR already done before\n");
	}

	printf("Initialization complete\n");

	Register_Read_FG(I2C_MASTER, DEVICE_NAME, Device_ID);
	uint16_t Dev_Name = (0xFFFF & (Device_ID[0] | Device_ID[1] << 8));

	return Dev_Name;



}

void Dynamic_Voltage_Neg_CV_Charg()
{
	Src_Cap_Req_union Source_PDO_Req;

	Register_Read_FG(I2C_MASTER, CURRENT, Current);
	float Curr_in_A = (0xFFFF & (Twos_Comp_to_unsigned_value((Current[1] << 8) | Current[0]))) *Current_factor; //printf("V_cell = %X\n",Vcell);
	printf("Curr_in_A = %f\n", (double)Curr_in_A);

	Register_Read_FG(I2C_MASTER, VCELL, Vcell);
	float Vcell_in_volt = (0xFFFF & ((Vcell[1] << 8) | Vcell[0])) * Voltage_factor; //printf("V_cell = %X\n",Vcell);
	printf("Vcell_in_V = %f\n", (double)Vcell_in_volt);

	float Charg_power = Vcell_in_volt * Curr_in_A;


	if (Charg_power < 15.0 && flag == FALSE)
	{
		Source_Cap_Req(1,  &Source_PDO_Req);
		printf("Power negotiated to 5V/3A\n");
//		MXC_Delay(5000000);
//    	MAX77986_Forward_Charging_ON(41500, 3000, 5500);
		flag = TRUE;
    }

}



