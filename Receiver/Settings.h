//*******  Setup hardware pin definitions here ! ***************
/*be sure to change the definitions to match your own setup. Some pins such as DIO1 and DIO2 
may not be in used by this sketch so they do not need to be connected and
should be set to -1.*/
//LORA Pinouts:
//GND  - BLACK   - GND
//+3V  - RED    - 3V
//RST  - BLUE   - PIN 8
//DI00 - YELLOW - PIN 2
//SCK - GREEN - D13
//MISO - BLACK - D12
////MOSI - YELLOW -  D11
//NSS - WHITE - D10

const int8_t NSS = 10;                          //select on LoRa device
const int8_t NRESET = 8;                        //reset on LoRa device Needs to be changed to allow PWM for motor 1! 
const int8_t DIO0 = 2;                          //DIO0 on LoRa device, used for RX and TX done
const int8_t DIO1 = -1;                         //DIO1 on LoRa device, normally not used so set to -1
const int8_t DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1
const int8_t LED1 = A4;                          //On board LED, logic high is on
#define LORA_DEVICE DEVICE_SX1278               //Change in case we change from 433/866/915 and so on. 

const int8_t OUTPUT1 = A5;                       //this output toggles when joystick switch is pressed on receiver
const uint16_t RXIdentity = 123;                //define a receiver number, the transmitter must use the same number
      
//*******  Setup LoRa Test Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 434000000;           //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_500;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF6;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting
//const uint8_t PacketLength = 5;                 //packet length is fixed
const uint8_t PacketLength = 8;                 //packet length is fixed
