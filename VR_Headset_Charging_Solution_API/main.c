////////////MAX32660 code for MAX77958 PD Negotiator////////////////////
//////////////Developed by Fahad Ahammad CAC APR////////////////////////

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "gpio.h"
#include "i2c.h"
#include "dma.h"
#include "led.h"
#include "FUNCTIONs.h"


// #define MASTERDMA


//#define I2C_SLAVE MXC_I2C1

#define I2C_FREQ 100000
#define I2C_BYTES 100

/***** Globals *****/

static uint8_t txdata[I2C_BYTES];
static uint8_t rxdata[I2C_BYTES];
uint8_t tx_buffer[50];
volatile uint8_t DMA_FLAG = 0;
volatile int I2C_FLAG;
volatile int txnum = 0;
volatile int txcnt = 0;
volatile int rxnum = 0;
volatile int num;


//Prints out human-friendly format to read txdata and rxdata
void printData(void)
{
    int i;
    printf("\n-->TxData: ");

    for (i = 0; i < sizeof(txdata); ++i) {
        printf("%02x ", txdata[i]);
    }

    printf("\n\n-->RxData: ");

    for (i = 0; i < sizeof(rxdata); ++i) {
        printf("%02x ", rxdata[i]);
    }

    printf("\n");

    return;
}


// *****************************************************************************
int main()
{

    int error = 0;

    //Setup the I2CM
    error = MXC_I2C_Init(I2C_MASTER, 1, 0);

    if (error != E_NO_ERROR) {
        printf("-->Failed master\n");
        return error;
    } else {
        printf("\n-->I2C Master Initialization Complete\n");
    }

    NVIC_EnableIRQ(I2C0_IRQn);

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

   //GPIO Interrupt Settings

    GPIO_Initialisation();

    Mask_Interrupts();

    Read_and_Clear_Interrupts();

    ///Union instances

    Current_Src_Cap_union curr_src_cap_combined;
    //Src_Cap_Req_union src_cap_req_combined;
    SNK_PDO_union snk_PDO_req_combined;
    uint32_t src_PDO1, src_PDO2, src_PDO3, src_PDO4, src_PDO5, src_PDO6, src_PDO7, src_PDO8;
	uint8_t Number_of_PDOs;
	uint8_t Selected_PDO;


	/**
	 *
	 * @brief   EZ initialization for MAX17262
	 *
	 * @parameters Design capacity, Termination Current, Empty Voltage, Recovery voltage, Charging voltage
	 *
	 */
    uint16_t Design_Capacity = 3400;
    uint16_t Terminatn_Current = 250;
    uint16_t Empty_Voltage = 3200;  //corresponds to zero SOC
    uint16_t Recovery_Voltage = 3880;
    uint16_t Charger_Voltage = 4200;

	MAX17262_EZ_Initialization(Design_Capacity, Terminatn_Current, Empty_Voltage, Recovery_Voltage, Charger_Voltage);  //change based on the battery they are using.
	/**
	 *
	 * @brief   Setting the initial configuration of MAX77958
	 *
	 * @parameters TrySNK mode, Power Role, MTP write, Moisture Detection, SRC Voltage and current
	 *
	 */
	MAX77958_Customer_Configuration(TrySNK_OFF, DRP, ENABLED, MOIST_DET_ON, 5000, 3000);

	//Calling twice is required due to interrupt clearing
	/**
	 *
	 * @brief   Reading the available source PDOs of the charger
	 *
	 * @return  Number of PDOs, Sink PDOs in hex value
	 */
	Current_Source_Cap(&curr_src_cap_combined, &Number_of_PDOs, &Selected_PDO, &src_PDO1, &src_PDO2, &src_PDO3, &src_PDO4, &src_PDO5, &src_PDO6, &src_PDO7, &src_PDO8);

	printf("Selected Source PDOs: %X\n", Selected_PDO);
    printf("Source_PDO1: %X\n", src_PDO1);
	printf("Source_PDO2:%X\n", src_PDO2);
	printf("Source_PDO3:%X\n", src_PDO3);
	printf("Source_PDO4:%X\n", src_PDO4);
	printf("Source_PDO5:%X\n", src_PDO5);
	printf("Source_PDO6:%X\n", src_PDO6);
	printf("Source_PDO7:%X\n", src_PDO7);
	printf("Source_PDO8:%X\n", src_PDO8);

	uint8_t Num_PDOs;

	uint32_t snk_PDO1, snk_PDO2, snk_PDO3, snk_PDO4, snk_PDO5, snk_PDO6;

	/**
	 * @brief   Reading the sink PDOs available
	 *
	 * @return  Number of PDOs, Sink PDOs in hex value
	 */
	Sink_PDO_Req(&snk_PDO_req_combined, &Num_PDOs, &snk_PDO1, &snk_PDO2, &snk_PDO3, &snk_PDO4, &snk_PDO5, &snk_PDO6);


	/**
	 * @brief   Setting Sink PDO for initial voltage configuration during charger plugin
	 *
	 * @param   memory write, No of PDOs, Sink PDO inputs in hex values (refer user guide for more details)
	 */
    Set_Sink_PDOs(MTP_Write, Position2, 0x1401912c, 0x0002D12C, 0x00000000, 0x00000000, 0x00000000, 0x00000000);

	printf("No: of Sink PDOs: %X\n", Num_PDOs);
	printf("Sink_PDO1: %X\n", snk_PDO1);
	printf("Sink_PDO2: %X\n", snk_PDO2);
	printf("Sink_PDO3: %X\n", snk_PDO3);
	printf("Sink_PDO4: %X\n", snk_PDO4);
	printf("Sink_PDO5: %X\n", snk_PDO5);
	printf("Sink_PDO6: %X\n", snk_PDO6);

	uint8_t Voltage_value, Current_value, PD_Status_value;
	/**
	 * @brief   Read the charger details
	 *
	 * @return  VBUS ADC Voltage read, Current limit, Status of connection & Charger
	 */
	Port_Detection_Status_Voltage_current_Read(&Voltage_value, &Current_value, &PD_Status_value);

    uint32_t Termination_Voltage = 41500; //in 4.5375V
    uint16_t Input_curr_lim  = 4000; //in mA
    uint16_t Fast_chg_curr = 5500;  //in mA

    MXC_Delay(100000);

    /**
     * @brief   Configuring MAX77962 Charger terminal voltage, Input USB Current limit, Fast charging current
     *
     * @param   Terminal voltage, Input current limit, Fast charging current
     *
     * @return  The I2C bus frequency in Hertz
     */
    MAX77986_Forward_Charging_ON(Termination_Voltage, Input_curr_lim, Fast_chg_curr);
    //Forward_Chg_OFF_Reverse_OTG_OFF_Boost_on(5000, 3000); //Initial State selection

    /**
     * @brief  Change the PDO Voltage
     *
     * @param   Position: Each position corresponds to each PDO voltage
     *
     * @return  No return
     */
//    Source_Cap_Req(Position4, &src_cap_req_combined);

    MXC_Delay(10000000);



//    uint8_t Chip_ID[1];
    while (1)

    {	    //GPIO polling and FG parameters reading
            //Note 1: When button is pressed with charger plugged in the LED will still be in blinking stage as Charg_In = 1
    	    //Note 2: If charger is plugged out and the button is pressed the LED stays on for 6seconds
    	    //Note 3: When the button is pressed and then the charger is plugged in immediately the LED will first blink for 6 seconds then later toggle in 0.5seconds

    	    /* Read state of the input pin. */
            if (MXC_GPIO_InGet(gpio_polling.port, gpio_polling.mask))
            {
                /* Input pin was high, do nothing. */
            }
            else
            {
              if (Charg_In == 0) //if button is pressed then the LEDs will be ON for a 6 second.
              {
               SOC_Indicator();    //Function for displaying the SOC via LEDs when button (GPIO5) is pressed
              }
            }

            if(Charg_In == 1)  //if charger is plugged in the LEDs will toggle at a rate of 0.5sec
            {
            	SOC_Indicator_Blink_Fn();

            }

            Dynamic_Voltage_Neg_CV_Charg();


            MXC_Delay(2000000);
     }


}









