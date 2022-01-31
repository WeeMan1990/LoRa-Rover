//********************************************************/
#include <SPI.h>
#include <SX127XLT.h>
#include "Settings.h"
#include <ProgramLT_Definitions.h>
#include "CytronMotorDriver.h"
#include <Servo.h> //TEST FÖR SERVO! 

SX127XLT LT;
//#define DEBUG //Add this variable if you want to enable debug and print data to serial. 

//Settings 
CytronMD motor1(PWM_PWM, 6, 9);  //RIGHT SIDE MOTOR - PIN D6 = M1A - WHITE || PIN D9 = M1B - GREEN
CytronMD motor2(PWM_PWM, 3, 5);  //LEFT  SIDE MOTOR - PIN D3 = M2B - YELLOW || PIN D5 = M2A - RED

Servo servotilt; //Servo 1 for camera Tilt
Servo servopan; //Servo 2 for camera Pan

uint8_t joystickX1value;                   //variable to read the value from the analog pin (RIGHT STICK) 
uint8_t joystickY1value;                   //variable to read the value from the analog pin (RIGHT STICK)
uint8_t joystickX2value;                   //variable to read the value from the analog pin (LEFT STICK)
uint8_t joystickY2value;                   //variable to read the value from the analog pin (LEFT STICK) 

uint8_t RXPacketL;                         //length of received packet
uint8_t RXPacketType;                      //type of received packet

void loop() 
{
  RXPacketL = LT.receiveSXBuffer(0, 0, WAIT_RX);   //returns 0 if packet error of some sort
  while (!digitalRead(DIO0));                      //wait for DIO0 to go high
  if ( LT.readIrqStatus() == (IRQ_RX_DONE + IRQ_HEADER_VALID))
  {
    packet_is_OK();
  }
  else
  {
    packet_is_Error();
  }
}

uint8_t packet_is_OK()
{
  //packet has been received, now read from the SX12xx Buffer using the same variable type and
  //order as the transmit side used.
  uint8_t TXIdentity;
  uint16_t pulseX1, pulseY1;
  uint8_t switchByte = 0xFF;              //this is the transmitted switch values, bit 0 = Switch0 etc
  uint16_t pulseX2, pulseY2;

  LT.startReadSXBuffer(0);                //start buffer read at location 0
  RXPacketType = LT.readUint8();          //read in the packet type
  TXIdentity = LT.readUint8();            //read in the transmitter number
  joystickX1value = LT.readUint8();       //this byte contains joystick pot AD X1 value sent
  joystickY1value = LT.readUint8();       //this byte contains joystick pot AD Y1 value sent
  switchByte = LT.readUint8();            //read in the Switch values
  joystickX2value = LT.readUint8();       //this byte contains joystick pot AD X2 value sent
  joystickY2value = LT.readUint8();       //this byte contains joystick pot AD Y2 value sent
  RXPacketL = LT.endReadSXBuffer();       //end buffer read

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

//Run motors!
   pulseX1 = map(joystickX1value, 0, 255, -255, 255);     //Re-map joystick to match I/O for MDD3A 
   motor1.setSpeed(pulseX1);
   pulseX2 = map(joystickX2value, 0, 255, -255, 255);     //Re-map joystick to match I/O for MDD3A 
   motor2.setSpeed(pulseX2);

  //Level the servos:
   pulseY1 = map(joystickY1value, 0, 255, 700, 2000);     //Re-map joystick to match I/O for MDD3A
   if (pulseY1<1200||pulseY1>1400) //Calibration for RIGHT controller stick dead center. 
  {
    servotilt.write(pulseY1);
  }
    else
  {
    servotilt.write(1337);
  }
  //Level servo 2
   pulseY2 = map(joystickY2value, 0, 255, 2300, 800);     //Re-map joystick to match I/O for MDD3A  
   if (pulseY2<1200||pulseY2>1600) //Calibration for RIGHT controller stick dead center. 
    {
      servopan.write(pulseY2);
    }
        else
    {
     servopan.write(1550);
 }
Serial.print("Tilt");
Serial.println(pulseY1);
Serial.print("Pan");
Serial.println(pulseY2);

  //actionOutputs-
  
  if (!bitRead(switchByte, 1))
  {
    digitalWrite(OUTPUT1, !digitalRead(OUTPUT1));          //Toggle Output state
  }
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
  }
  Serial.println();
}

void setupOutputs()
{
  //configure the output pins, if a pin is defiend in 'Settings.h' as -1, its not configured, so stays as input
  if (OUTPUT1  >= 0)
  {
    pinMode(OUTPUT1, OUTPUT);
  }
}
void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    //digitalWrite(LED1, HIGH);
    delay(delaymS);
    //digitalWrite(LED1, LOW);
    delay(delaymS);
  }
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
      servotilt.attach(4); 
      servopan.attach(7);
  pinMode(LED1, OUTPUT);
  //led_Flash(2, 125);
  setupOutputs();
  Serial.begin(115200);

  SPI.begin();
  
  if (LT.begin(NSS, NRESET, DIO0, DIO1, DIO2, LORA_DEVICE))
  {
    Serial.println(F("Link Ready!"));
    //led_Flash(2, 125);
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
  Serial.println(F("LoRa Rover Receiver is good to go!"));
  Serial.println();
}
