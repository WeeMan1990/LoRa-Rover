//**************************************************************************
#include <SPI.h>
#include <SX127XLT.h>
#include "Settings.h"
#include <ProgramLT_Definitions.h>

SX127XLT LT;

#include "PinChangeInterrupt.h"             //https://github.com/NicoHood/PinChangeInterrupt

uint32_t TXpacketCount;
uint8_t TXPacketL;

uint8_t joystickX1value;                    //variable to read the value from the analog pin 0 
uint8_t joystickY1value;                    //variable to read the value from the analog pin 1
uint8_t joystickX2value;                    //variable to read the value from the analog pin 2
uint8_t joystickY2value;                    //variable to read the value from the analog pin 3

volatile bool switch1flag = false;

#define DEBUG                               //(remove the two // at the beggining) for debug output
void loop()
{
  uint8_t switchByte = 0xFF;
//Below we read the joysticks.
//Stick 1 (X1/Y1) controls the RIGHT side motor (X Axis) and camera tilt (Y Axis).
//Stick 2 (X2/Y2) controls the LEFT  side motor (X Axis) and camera pan (Y Axis)
  joystickX1value = (uint8_t) (analogRead(joystickX1) / 4) ;       //read the RIGHT (X1) pot, turn 0-1023 into 0 to 255
  joystickY1value = (uint8_t) (analogRead(joystickY1) / 4);        //read the RIGHT (Y1) pot, turn 0-1023 into 0 to 255
  joystickX2value = (uint8_t) (analogRead(joystickX2) / 4) ;       //read the LEFT (X2) pot, turn 0-1023 into 0 to 255
  joystickY2value = (uint8_t) (analogRead(joystickY2) / 4);        //read the LEFT (X2) pot, turn 0-1023 into 0 to 255
  
  if (switch1flag)
  {
    bitClear(switchByte, 1);                       //if the switch is down clear the bit
    digitalWrite(LED1, HIGH);                      //turn on LED as switch indicator (Not using at the moment) 
    switch1flag = false;
  }
  if (!sendJoystickPacket(joystickX1value, joystickY1value, switchByte, joystickX2value, joystickY2value))
  {
    Serial.print(F("Send Error - IRQreg,"));
    Serial.print(LT.readIrqStatus(), HEX);
  }
}

uint8_t sendJoystickPacket(uint16_t X1value, uint16_t Y1value, uint8_t switches, uint16_t X2value, uint16_t Y2value)
{
  //The SX12XX buffer is filled with variables of a known type and in a known sequence.
  //Make sure the receiver uses the same variable types and sequence to read variables out of the receive buffer.
  //uint8_t len;
  uint32_t packetStartmS, packettimemS;

  LT.startWriteSXBuffer(0);                      //start the write packet to buffer process
  LT.writeUint8(RControl1);                      //this is the packet type
  LT.writeUint8(TXIdentity);                     //this value represents the transmitter number (123 set in settings.h!) 
  LT.writeUint8(X1value);                        //this byte contains joystick pot ADC X1 value to be sent (Pin A0)
  LT.writeUint8(Y1value);                        //this byte contains joystick pot ADC Y1 value to be sent (Pin A1) 
  LT.writeUint8(switches);                       //switches value - Whichever whitch one wants (RH Joystick for now) 
  LT.writeUint8(X2value);                        //this byte contains joystick pot ADC X2 value to be sent (Pin A2)
  LT.writeUint8(Y2value);                        //this byte contains joystick pot ADC Y2 value to be sent (Pin A3)
  LT.endWriteSXBuffer();                         //close the packet, there are 8 bytes to send (7 for now if im right) 

  //now transmit the packet, 10 second timeout, and wait for it to complete sending
  packetStartmS = millis();
  TXPacketL = LT.transmitSXBuffer(0, PacketLength, 10000, TXpower, WAIT_TX);
  packettimemS = millis() - packetStartmS;

#ifdef DEBUG
//If Define debug is added, print debug on the switches so we know they are reading values as expected. 
//DEBUG STICK 1: 
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

  digitalWrite(LED1, LOW);                         //LED off, may have been on due to switch press
  delay(packettimemS * 9);                         //delay for 9 times packet transmit time to ensure 10% duty cycle
  return TXPacketL;                                //TXPacketL will be 0 if there was an error sending
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}


void attachInterrupts()
{
  if (SWITCH1  >= 0)
  {
    attachPCINT(digitalPinToPCINT(SWITCH1), wake1, FALLING);
    switch1flag = false;
  }
}


void detachInterrupts()
{
  if (SWITCH1  >= 0)
  {
    detachPCINT(digitalPinToPCINT(SWITCH1));
  }
}


void wake1()
{
  switch1flag = true;
}


void setupSwitches()
{
  if (SWITCH1  >= 0)
  {
    pinMode(SWITCH1, INPUT_PULLUP);
  }
}


void setupLoRa()
{
  //this setup is used so that the implicit packet type,LORA_PACKET_FIXED_LENGTH, is used  
  LT.setMode(MODE_STDBY_RC);                              //go to standby mode to configure device
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


void setup()

{
  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);
  setupSwitches();
  Serial.begin(115200);
  SPI.begin();

  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE))
  {
    led_Flash(2, 125);
  }
  else
  {
    Serial.println(F("Device error"));
    while (1)
    {
      led_Flash(50, 50);
    }
  }

  //this function call sets up the device for LoRa using the settings from the Settings.h file
  setupLoRa();

  attachInterrupts();

  Serial.println();
  LT.printModemSettings();                                //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println(F("LoRa Rover-TX all set!"));
  Serial.println();
}
