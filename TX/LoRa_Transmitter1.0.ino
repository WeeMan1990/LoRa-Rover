#include <SPI.h>
#include <SX127XLT.h>
#include "Settings.h"
#include <ProgramLT_Definitions.h>
#include "PinChangeInterrupt.h"
SX127XLT LT;
//#define DEBUG                 //Enable or Disable debugging by removing or adding //

uint8_t TXPacketL;
uint32_t TXpacketCount;
uint8_t joystickX1value;      //variable to read the value from the analog pin 0 
uint8_t joystickY1value;      //variable to read the value from the analog pin 1
uint8_t joystickX2value;      //variable to read the value from the analog pin 2
uint8_t joystickY2value;      //variable to read the value from the analog pin 3         
int switch1State = 0;         // variable for reading Switch 1 status (Servocontrol on/off)
int switch2State = 0;         // variable for reading Switch 2 status (Gearstate)
int switch3State = 0;         // variable for reading Switch 3 status (IR Disable)
int switch4State = 0;         // variable for reading Switch 4 status (NO FUNCTION YET)
//-------------------------------------------------------------------------------------------------------------
void loop()
{
  switch1State = digitalRead(switch1);                             //Read the state of switch 1 (Used to release servocontrol latch)
  switch2State = digitalRead(switch2);                             //Read the state of switch 2
  switch3State = digitalRead(switch3);                             //Read the state of switch 3
  switch4State = digitalRead(switch4);                             //Read the state of switch 3
  uint8_t switchByte = 0xFF;
  joystickX1value = (uint8_t) (analogRead(joystickX1) / 4);        //read the RIGHT (X1) pot, turn 0-1023 into 0 to 255
  joystickY1value = (uint8_t) (analogRead(joystickY1) / 4);        //read the RIGHT (Y1) pot, turn 0-1023 into 0 to 255
  joystickX2value = (uint8_t) (analogRead(joystickX2) / 4);        //read the LEFT (X2) pot, turn 0-1023 into 0 to 255
  joystickY2value = (uint8_t) (analogRead(joystickY2) / 4);        //read the LEFT (X2) pot, turn 0-1023 into 0 to 255
  //Read switches and set their bits! 
  if (switch1State == 1) {                                         //Change 1 to 0 vice versa depending on the switchstate 
    bitClear(switchByte, 0);                                       //if the switch is down set bit 0 to 0 (Servoadjustment will available) 
  }
  if (switch2State == 1) {                                         //Change 1 to 0 vice versa depending on the switchstate 
    bitClear(switchByte, 1);                                       //if the switch is down set bit 1 to 0 (Relay will switch from low to high gear) 
  }
  if (switch3State == 1) {                                         //Change 0 to 1 vice versa depending on the switchstate 
    bitClear(switchByte, 2);                                       //if the switch is down set bit 2 to 0 (Relay will toggle relay for IR) 
  }
   if (switch4State == 1) {                                        //Change 0 to 1 vice versa depending on the switchstate 
    bitClear(switchByte, 3);                                       //if the switch is down set bit 3 to 0 (Not used yet on RX Side). 
  }
 //---------------------------------------------------------------------------------------------------------------------- 
  if (!sendJoystickPacket(joystickX1value, joystickY1value, joystickX2value, joystickY2value, switchByte))
  {
    Serial.print(F("Send Error - IRQreg,"));
    Serial.print(LT.readIrqStatus(), HEX);
  }
}
uint8_t sendJoystickPacket(uint16_t X1value, uint16_t Y1value, uint16_t X2value, uint16_t Y2value, uint8_t switches)
{
  uint8_t len;
  uint32_t packetStartmS, packettimemS;
  LT.startWriteSXBuffer(0);                      //start the write packet to buffer process
  LT.writeUint8(RControl1);                      //this is the packet type
  LT.writeUint8(TXIdentity);                     //this value represents the transmitter number (123 set in settings.h!) 
  LT.writeUint8(X1value);                        //this byte contains joystick pot ADC X1 value to be sent (Pin A0)
  LT.writeUint8(Y1value);                        //this byte contains joystick pot ADC Y1 value to be sent (Pin A1) 
  LT.writeUint8(X2value);                        //this byte contains joystick pot ADC X2 value to be sent (Pin A2)
  LT.writeUint8(Y2value);                        //this byte contains joystick pot ADC Y2 value to be sent (Pin A3)
  LT.writeUint8(switches);                       //this byte contains switch-values (Bitwise)
  LT.endWriteSXBuffer();                         //close the packet
  //now transmit the packet, timeout, and wait for it to complete sending
  packetStartmS = millis();
  TXPacketL = LT.transmitSXBuffer(0, PacketLength, 10000, TXpower, WAIT_TX);
  packettimemS = millis() - packetStartmS;

//-------------------------------------------------------------------------------------------------------------
#ifdef DEBUG
//DEBUG STICK 1
  Serial.print(TXIdentity);
  Serial.print(F(",X1,"));
  Serial.print(joystickX1value);
  Serial.print(F(",Y1,"));
  Serial.print(joystickY1value);
  Serial.print(F(","));
  Serial.print(switches, BIN);
  Serial.print(F(","));
  Serial.print(packettimemS);
  Serial.print(F("mS"));
  Serial.println();
//DEBUG STICK 2: 
  Serial.print(TXIdentity);
  Serial.print(F(",X2,"));
  Serial.print(joystickX2value);
  Serial.print(F(",Y2,"));
  Serial.print(joystickY2value);
  Serial.print(F(","));
  Serial.print(switches, BIN);
  Serial.print(F(","));
  Serial.print(packettimemS);
  Serial.print(F("mS"));
  Serial.println();
#endif

  delay(packettimemS * 9);                         //delay for 9 times packet transmit time to ensure 10% duty cycle
  return TXPacketL;                                //TXPacketL will be 0 if there was an error sending
}
//-------------------------------------------------------------------------------------------------------------
void setupLoRa()
{
  //this setup is used so that the implicit packet type,LORA_PACKET_FIXED_LENGTH, is used  
  LT.setMode(MODE_STDBY_RC);                                                                   //go to standby mode to configure device
  LT.setPacketType(PACKET_TYPE_LORA);                                                          //set for LoRa transmissions
  LT.setRfFrequency(Frequency, Offset);                                                        //set the operating frequency
  LT.calibrateImage(0);                                                                        //run calibration after setting frequency
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);                     //set LoRa modem parameters
  LT.setBufferBaseAddress(0x00, 0x00);                                                         //where in the SX buffer packets start, TX and RX
  LT.setPacketParams(8, LORA_PACKET_FIXED_LENGTH, PacketLength, LORA_CRC_ON, LORA_IQ_NORMAL);  //set packet parameters
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);                                                   //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34
  LT.setHighSensitivity();                                                                     //set for highest sensitivity at expense of slightly higher LNA current
  //This is the typical IRQ parameters set, actually excecuted in the transmit function
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);                                        //set for IRQ on TX done
}
//-------------------------------------------------------------------------------------------------------------
void setup()
{
  pinMode(switch1, INPUT); //Define pin as input
  pinMode(switch2, INPUT); //Define pin as input
  pinMode(switch3, INPUT); //Define pin as input
  pinMode(switch4, INPUT); //Define pin as input
  Serial.begin(115200);
  SPI.begin();
  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE))
  {
    Serial.println(F("All good, ready for launch!"));
  }
  else
  {
    Serial.println(F("Device error"));
  }
  //this function call sets up the device for LoRa using the settings from the Settings.h file
  setupLoRa();
  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println(F("LoRa Rover-TX all set!"));
  Serial.println();
}
