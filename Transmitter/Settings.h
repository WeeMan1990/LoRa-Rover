//*******  Setup hardware pin definitions here ! ***************
//be sure to change the definitiosn to match your own setup. Some pins such as DIO1,
//DIO2, may not be in used by this sketch so they do not need to be connected and
//should be set to -1.*/

//
const int8_t NSS = 10;                          //select on LoRa device
const int8_t NRESET = 9;                        //reset on LoRa device
const int8_t DIO0 = 2;                          //DIO0 on LoRa device, used for RX and TX
const int8_t DIO1 = -1;                         //DIO1 on LoRa device, normally not used so set to -1
const int8_t DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1

const int8_t LED1 = 8;                          //On board LED, logic high is on //Not connected on the board ATM. 

#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using, change if module is changed to 866/916 MHz

const int8_t joystickX1 = A0;                   //analog pin for the joystick 1 X pot
const int8_t joystickY1 = A1;                   //analog pin for the joystick 1 Y pot
const int8_t SWITCH1 = 3;                       //switch on joystick, set to -1 if not used
const int8_t joystickX2 = A2;                   //analog pin for the joystick 1 X pot
const int8_t joystickY2 = A3;                   //analog pin for the joystick 1 Y pot
//const int8_t SWITCH2 = 4;                     //switch on joystick 2 (not implemented at all yet)
  
const uint32_t TXIdentity = 123 ;               //define a transmitter number, the receiver must use the same number

//*******  Setup LoRa TEST Parameters Here ! ***************
const uint32_t Frequency = 434000000;           //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes
const uint8_t Bandwidth = LORA_BW_500;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF6;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting
const uint8_t PacketLength = 8;                 //packet length is fixed 
const int8_t TXpower = 10;                      //LoRa transmit power in dBm
