//*******  Setup hardware pin definitions here ! ***************
/*Be sure to change the definitions to match your own setup. Some pins such as DIO1,
DIO2, may not be in used by this sketch so they do not need to be connected and
should be set to -1.*/
// Arduino NANO
//   LoRa SCK - D13|    |D12 - LoRa MISO                   
//   To 3v Rail 3v3|    |D11 - LoRa MOSI
//              REF|    |D10 - LoRa NSS
//   Stick 1 X - A0|    |D9  - LoRa RST
//   Stick 1 Y - A1|    |D8
//   Stick 2 X - A2|    |D7
//   Stick 2 Y - A3|    |D6 - Switch 4 (10K Resistor to GND is needed)
//               A4|    |D5 - Switch 3 (10K Resistor to GND is needed)
//               A5|    |D4 - Switch 2 (10K Resistor to GND is needed)
//               A6|    |D3 - Switch 1 (10K Resistor to GND is needed)
//               A7|    |D2 - LoRa DI00
//   To 5v Rail  5V|    |GND - GND to rails
//              RST|    |RST
//Battery GND - GND|    |RX0
//Battery VCC - VIN|    |TX1
// 3 Volt rail connections - Lora and Switches! 
// 5 Volt rail connections - Joysticks

const int8_t NSS = 10;                          //select on LoRa device
const int8_t NRESET = 9;                        //reset on LoRa device
const int8_t DIO0 = 2;                          //DIO0 on LoRa device, used for RX and TX
const int8_t DIO1 = -1;                         //DIO1 on LoRa device, normally not used so set to -1
const int8_t DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1
const uint32_t TXIdentity = 123 ;               //define a transmitter number, the receiver must use the same number

#define LORA_DEVICE DEVICE_SX1278               //this is the device we are using, change if module is changed to 866/916 MHz

const int8_t joystickX1 = A0;                   //analog pin for the joystick 1 X pot
const int8_t joystickY1 = A1;                   //analog pin for the joystick 1 Y pot
const int8_t joystickX2 = A2;                   //analog pin for the joystick 1 X pot
const int8_t joystickY2 = A3;                   //analog pin for the joystick 1 Y pot
const int switch1 = 3;                          //Switch 1 (Servo lock ON/OFF) This do not have any output pins on the receiver side.
const int switch2 = 4;                          //Switch 2 (Switches the main relay so the MDD3A gets 12v instead of 7 (High speed gear))
const int switch3 = 5;                          //Switch 3 (Switches the second relay chages it state and enables IR, since IR is annoying in certain low light conditions.)
const int switch4 = 6;                          //Switch 4 (NO FUNCTION YET)

//LoRa Modem Parameters
const uint32_t Frequency = 484500000;           //frequency of transmissions
const uint32_t Offset = 0;                      //offset frequency for calibration purposes
const uint8_t Bandwidth = LORA_BW_250;          //LoRa bandwidth (Changed from 500 to 250 for testing) 
const uint8_t SpreadingFactor = LORA_SF6;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting
const uint8_t PacketLength = 8;                 //packet length is fixed 
const int8_t TXpower = 18;                      //LoRa transmit power in dBm

//TX Settings: 
/* 
 *      int8_t _NSS, _NRESET, _DIO0, _DIO1, _DIO2;
    uint8_t _RXPacketL;             //length of packet received
    uint8_t _RXPacketType;          //type number of received packet
    uint8_t _RXDestination;         //destination address of received packet
    uint8_t _RXSource;              //source address of received packet
    uint8_t  _TXPacketL;            //length of transmitted packet
    uint16_t _IRQmsb;               //for setting additional flags
    uint8_t _Device;                //saved device type
    int8_t _TXDonePin;              //the pin that will indicate TX done
    int8_t _RXDonePin;              //the pin that will indicate RX done
    uint8_t _UseCRC;                //when packet parameters are set this flag is set if CRC on packets in use
    int8_t _RXEN, _TXEN;            //for modules that have RX TX pin switching
    uint8_t _PACKET_TYPE;           //used to save the set packet type
    uint8_t _freqregH, _freqregM, _freqregL;                 //the registers values for the set frequency
    uint8_t _ShiftfreqregH, _ShiftfreqregM, _ShiftfreqregL;  //register values for shifted frequency, used in FSK RTTY etc
    uint32_t _savedFrequency;       //when setRfFrequency() is used the set frequency is saved
    int32_t _savedOffset;           //when setRfFrequency() is used the set offset is saved
    uint8_t _ReliableErrors;        //Reliable status byte
    uint8_t _ReliableFlags;         //Reliable flags byte
    uint8_t _ReliableConfig;        //Reliable config byte

    //Constant names for bandwidth settings
#define    LORA_BW_500                              144  //actual 500000hz
#define    LORA_BW_250                              128  //actual 250000hz
#define    LORA_BW_125                              112  //actual 125000hz
#define    LORA_BW_062                              96   //actual  62500hz 
#define    LORA_BW_041                              80   //actual  41670hz
#define    LORA_BW_031                              64   //actual  31250hz 
#define    LORA_BW_020                              48   //actual  20830hz
#define    LORA_BW_015                              32   //actual  15630hz
#define    LORA_BW_010                              16   //actual  10420hz 
#define    LORA_BW_007                              0    //actual   7810hz

//for SX127x
#define    LORA_SF6                                 0x06
#define    LORA_SF7                                 0x07
#define    LORA_SF8                                 0x08
#define    LORA_SF9                                 0x09
#define    LORA_SF10                                0x0A
#define    LORA_SF11                                0x0B
#define    LORA_SF12                                0x0C
 */
