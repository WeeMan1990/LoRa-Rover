//*******  Setup hardware pin definitions here ! ***************
/*be sure to change the definitions to match your own setup. Some pins such as DIO1 and DIO2 
may not be in used by this sketch so they do not need to be connected and
should be set to -1.*/
// Arduino NANO
//     LoRa SCK - D13|    |D12 - LoRa MISO                    
//                3v3|    |D11 - LoRa MOSI
//                REF|    |D10 - LoRa NSS
//Switch1 output - A0|    |D9  - M1B on MDD3A
//Switch2 output - A1|    |D8  - LoRa RST 
//Switch3 output - A2|    |D7  - PAN Servo
//Switch4 output - A3|    |D6  - M1A on MDD3a 
//                 A4|    |D5  - M2A on MDD3a 
//                 A5|    |D4  - TILT Servo 
//                 A6|    |D3  - M2B on MDD3a 
//                 A7|    |D2  - LoRa DI00
//                 5V|    |GND -
//                RST|    |RST -
//                GND|    |RX0 - 
//                VIN|    |TX1 -
//LORA WIRING:
//GND  - GND - BLACK ||  RST  - PIN 8 - BLUE  || SCK - D13 - GREEN || MISO - D12 - BLACK  ||
//+3V  - 3v3 - RED   || DI00 - PIN 2 - Yellow || NSS - D10 - WHITE || MOSI - D11 - Yellow ||
//MDD3A WIRING COLORS: 
//RH Motor: M1B (REVERSE) - GREEN  - PIN 9 (490HZ PWM) || M1A (FWD) - WHITE - PIN 6 (980HZ PWM)
//LH Motor: M2B (REVERSE) - YELLOW - PIN 3 (490HZ PWM) || M2A (FWD) - RED   - PIN 5 (980HZ PWM)

const int8_t NSS = 10;                          //select on LoRa device
const int8_t NRESET = 8;                        //reset on LoRa device
const int8_t DIO0 = 2;                          //DIO0 on LoRa device, used for RX and TX done
const int8_t DIO1 = -1;                         //DIO1 on LoRa device, normally not used so set to -1
const int8_t DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1
#define LORA_DEVICE DEVICE_SX1278               //Change in case we change from 433/866/915 and so on. 

//Below is settings for switches outputs. This is done with bitread, ie if byte is 11111101 switch 1 will be enabled. If bit 3 is 1, switch 2 is enabled. 
//The first bit therefore needs to be ignored as this is used inside the packet_is_OK function to lock or unlock servo control.
//To make life easier, the names are set as ignore bit or output bit. 
const int8_t IGNOREBIT0 = -1;                   //This is just here to be able to ignore the first bit as this is used to latch the servo.
const int8_t OUTPUTBIT1 = A0;                   //If bit 1 is 0 The main relay switches so the MDD3a gets 12v instead of 8 (High speed gear)
const int8_t OUTPUTBIT2 = A1;                   //If bit 2 is 0 the second relay chages it state and enables IR, since IR is annoying in certain low light conditions.
const int8_t OUTPUTBIT3 = A2;                   //NOT USED YET
const int8_t IGNOREBIT5 = -1;                   //NOT USED YET
//const int8_t OUTPUTBIT4 = A3;                 //NOT INCLUDED AT ALL. 
const uint16_t RXIdentity = 123;                //Define a receiver number, the transmitter must use the same number
int PanCenter = 1460;                           //Calibration for center position, based on Y2 map (Higher value is LEFT lower is RIGHT)
int TiltCenter = 1350;                          //Calibration for center position, based on Y1 map (Higher value is downwards lower is upwards)

//LoRa Modem Parameters
const uint32_t Frequency = 484500000;           //Frequency of transmissions
const uint32_t Offset = 0;                      //Offset frequency for calibration purposes
const uint8_t Bandwidth = LORA_BW_250;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF6;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting
const uint8_t PacketLength = 8;                 //packet length is fixed
