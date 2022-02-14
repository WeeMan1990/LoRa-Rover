
//********************************************************/
#include <SPI.h>
#include <SX127XLT.h>
#include "Settings.h"
#include <ProgramLT_Definitions.h>
#include "CytronMotorDriver.h"
#include <Servo.h>
SX127XLT LT;

//#define DEBUG              //Add line in case stick input needs to be checked (RX side)
//#define SERVODBG           //Add line in case servos needs to be checked (I/O RX Side)
//#define RSSICHK            //Add line in case RSSI needs to be checked

//Setups
Servo servotilt;                      //Servo 1 for camera Tilt
Servo servopan;                       //Servo 2 for camera Pan
CytronMD motor1(PWM_PWM, 6, 9);       //M1B RH (REV) - GREEN  - PIN 9 (490HZ PWM) || M1A RH (FWD) - WHITE - PIN 6 (980HZ PWM) 
CytronMD motor2(PWM_PWM, 5, 3);       //M2B LH (REV) - YELLOW - PIN 3 (490HZ PWM) || M2A LH (FWD) - RED   - PIN 5 (980HZ PWM)
uint8_t joystickX1value;              //variable to read the value from the analog pin (RIGHT STICK) 
uint8_t joystickY1value;              //variable to read the value from the analog pin (RIGHT STICK)
uint8_t joystickX2value;              //variable to read the value from the analog pin (LEFT STICK)
uint8_t joystickY2value;              //variable to read the value from the analog pin (LEFT STICK) 
uint8_t RXPacketL;                    //length of received packet
uint8_t RXPacketType;                 //type of received packet

void loop() 
{
  RXPacketL = LT.receiveSXBuffer(0, 0, WAIT_RX);                //returns 0 if packet error of some sort
  while (!digitalRead(DIO0));                                   //wait for DIO0 to go high
  if ( LT.readIrqStatus() == (IRQ_RX_DONE + IRQ_HEADER_VALID))
  {
    packet_is_OK();                                            //Run function packet_is_OK assuming that the packet is OK
  }
  else
  {
    packet_is_Error();                                         //Run function packet_is_error if there are issues. 
  }
}

uint8_t packet_is_OK()
{
  //packet has been received, now read from the SX12xx Buffer using the same variable type and order as the transmit side used.
  uint8_t TXIdentity;
  uint16_t pulseX1, pulseY1;              //Transmitted values of the first joystick
  uint8_t switchByte = 0xFF;              //this is the transmitted switch values, bit 0 = Switch0 etc
  uint16_t pulseX2, pulseY2;              //Transmitted values of the Second joystick
  LT.startReadSXBuffer(0);                //start buffer read at location 0
  RXPacketType = LT.readUint8();          //read in the packet type
  TXIdentity = LT.readUint8();            //read in the transmitter number
  joystickX1value = LT.readUint8();       //this byte contains joystick pot AD X1 value sent
  joystickY1value = LT.readUint8();       //this byte contains joystick pot AD Y1 value sent
  joystickX2value = LT.readUint8();       //this byte contains joystick pot AD X2 value sent
  joystickY2value = LT.readUint8();       //this byte contains joystick pot AD Y2 value sent
  switchByte = LT.readUint8();            //read in the Switch values
  RXPacketL = LT.endReadSXBuffer();       //end buffer read
  if (RXPacketType != RControl1)
  {
    Serial.print(F("Packet type "));
    Serial.println(RXPacketType);
    return 0;
  }
  if (TXIdentity != RXIdentity)
  {
    Serial.print(F("TX"));
    Serial.print(TXIdentity);
    Serial.println(F("?"));
    return 0;
  }

   //Remap input and run the motors! 
   pulseX1 = map(joystickX1value, 0, 255, -255, 255);     //Re-map joystick to match I/O for MDD3A 
   motor1.setSpeed(pulseX1);                              //Set motor 1 speed (RIGHT SIDE TRACK)               
   pulseX2 = map(joystickX2value, 0, 255, -255, 255);     //Re-map joystick to match I/O for MDD3A
   motor2.setSpeed(pulseX2);                              //Set motor 2 speed (LEFT SIDE TRACK) 
  //Level the servos if byte 0 is released:               //
  if (!bitRead(switchByte, 0))                            //Only level servos if the first bit in the 8 bit message is set to 0! 
  {                                                       //
   pulseY1 = map(joystickY1value, 0, 255, 2300, 800);     //If value from Y1 is 0 the converted value is 2300, 2300 is downwards and 800 is upwards
   if (pulseY1<1500||pulseY1>1600)  {                     //Calibration for RIGHT controller stick Y Axis to avoid servo jitter.
   servotilt.write(pulseY1); }                            //Set tilt angle to the Y1 Value
     else  {                                              //
   servotilt.write(TiltCenter);                           //Calibration for center position, based on Y1 map (Higher value is downwards lower is upwards)
  }
   //Level servo 2
   pulseY2 = map(joystickY2value, 0, 255, 2200, 800);     //If value from Y2 is 0 the converted value is 2200, 2200 is LEFT and 800 is RIGHT.
   if (pulseY2<1200||pulseY2>1600)  {                     //Calibration for LEFT controller stick Y Axis to avoid servo jitter.  
   servopan.write(pulseY2);  }                            //Set pan angle to the Y2 Value
    else  {                                               //
   servopan.write(PanCenter);                             //Sets pan to center, the center position value can be adjusted in settings.h
  }
 }
 //Set outputs based on bit array in the byte
    byte pins[5] = { IGNOREBIT0, OUTPUTBIT1, OUTPUTBIT2, OUTPUTBIT3, IGNOREBIT5} ; //Bit 0, bit 1, bit 2, bit 3
    for (byte i = 0 ; i < sizeof(pins) ; i++)
    digitalWrite (pins[i], ( switchByte & (1<<i)) != 0 ? HIGH : LOW) ;
 
//-------------------------------------------------------------------------------------------------------------
//DEBUG SECTION - DEBUG SECTION - DEBUG SECTION - DEBUG SECTION - DEBUG SECTION - DEBUG SECTION - DEBUG SECTION
//-------------------------------------------------------------------------------------------------------------
#ifdef SERVODBG
Serial.print("PulseY1 received: ");
Serial.println(joystickY1value);
Serial.print("PulseY1 converted: ");
Serial.println(pulseY1);
Serial.print("PulseY2 received: ");
Serial.println(joystickY2value);
Serial.print("PulseY2 Converted: ");
Serial.println(pulseY2);
#endif

#ifdef DEBUG
//Print Right side stick
  Serial.print(TXIdentity);
  Serial.print(F(",X1,"));
  Serial.print(joystickX1value);
  Serial.print(F(",Y1,"));
  Serial.print(joystickY1value);
  Serial.print(F(","));
  Serial.print(switchByte, BIN);
  Serial.println();
//Print Left side stick
  Serial.print(TXIdentity);
  Serial.print(F(",X2,"));
  Serial.print(joystickX2value);
  Serial.print(F(",Y2,"));
  Serial.print(joystickY2value);
  Serial.print(F(","));
  Serial.print(switchByte, BIN);
  Serial.println();
#endif

#ifdef RSSICHK
int16_t CheckRSSI;
CheckRSSI = LT.readPacketRSSI();
Serial.println(CheckRSSI);
#endif

  return RXPacketL;
}
void packet_is_Error()
{
  uint16_t IRQStatus;
  int16_t PacketRSSI;
  IRQStatus = LT.readIrqStatus();
  if (IRQStatus & IRQ_RX_TIMEOUT)
  {
    Serial.print(F("RXTimeout"));
  }
  else
  {
    PacketRSSI = LT.readPacketRSSI();                        //read the signal strength of the received packet
    Serial.print(F("Err,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm"));
    motor1.setSpeed(0); 
    motor2.setSpeed(0); 
  }
  Serial.println();
}

//------------------------------------------------------------------------------------------------------------------------------------
void setupLoRa()
{
  //this setup is used so as the implicit packet type,LORA_PACKET_FIXED_LENGTH, is used  
  LT.setMode(MODE_STDBY_RC);                              //got to standby mode to configure device
  LT.setPacketType(PACKET_TYPE_LORA);                     //set for LoRa transmissions
  LT.setRfFrequency(Frequency, Offset);                   //set the operating frequency
  LT.calibrateImage(0);                                   //run calibration after setting frequency
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);  //set LoRa modem parameters
  LT.setBufferBaseAddress(0x00, 0x00);                    //where in the SX buffer packets start, TX and RX
  LT.setPacketParams(8, LORA_PACKET_FIXED_LENGTH, PacketLength, LORA_CRC_ON, LORA_IQ_NORMAL);  //set packet parameters
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);              //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34
  LT.setHighSensitivity();                                //set for highest sensitivity at expense of slightly higher LNA current
  //This is the typical IRQ parameters set, actually excecuted in the transmit function
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);   //set for IRQ on TX done
}
//------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
 pinMode(OUTPUTBIT1, OUTPUT); //If bit 1 sets to 1, toggle output
 pinMode(OUTPUTBIT2, OUTPUT); //If bit 2 sets to 1, toggle output      
 pinMode(OUTPUTBIT3, OUTPUT); //If bit 3 sets to 1, toggle output
 //pinMode(OUTPUTBIT4, OUTPUT); //If bit 4 sets to 1, toggle output
  servotilt.attach(4); 
  servopan.attach(7);
  Serial.begin(115200);
  SPI.begin();
  
  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE))
  {
    Serial.println(F("Link Ready!"));
  }
  else
  {
    Serial.println(F("Device error"));
  }
  //this function call sets up the device for LoRa using the settings from the Settings.h file
  setupLoRa();
  Serial.println();
  LT.printModemSettings();                                //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println(F("LoRa Rover is good to go!"));
  Serial.println();
  //Run system 0
  motor1.setSpeed(0); 
  motor2.setSpeed(0);
  //Servos need to be set here to set them to center, without it, the servos wont align until bit 0 is cleared! 
  servotilt.write(TiltCenter);
  servopan.write(PanCenter); 
}
