// SDL_Arduino_WXLink_LoRa_Rx
// SwitchDoc Labs March 2017
//

#define SOFTWAREVERSION 005

#include <SoftwareSerial.h>

#include "MemoryFree.h"

#include <Wire.h>

#include "Crc16.h"

//Crc 16 library (XModem)
Crc16 crc;

#define LED 13
#include <RH_RF95.h>


// Singleton instance of the radio driver

SoftwareSerial SoftSerial(8, 9); // TX, RX
RH_RF95 <SoftwareSerial>rf95(SoftSerial);

byte buffer[75];
byte lastGoodMessage[64];
byte buflen = 0;

long consecutiveGoodMessages;
long lastGoodMessageID;
long goodMessages;
long badMessages;

int toggle = 0;


void printBuffer(byte *buffer, int buflen)
{
  int i;
  for (i = 0; i < buflen; i++)
  {
    Serial.print("i=");
    Serial.print(i);
    Serial.print(" | ");
    Serial.println(buffer[i], HEX);
  }

}


int convert2BytesToInt(byte *buffer, int bufferStart)
{

  union u_tag {
    byte b[2];
    int fval;
  } u;

  u.b[0] = buffer[bufferStart];
  u.b[1] = buffer[bufferStart + 1];


  return u.fval;

}

long convert4BytesToLong(byte *buffer, int bufferStart)
{

  union u_tag {
    byte b[4];
    long fval;
  } u;

  u.b[0] = buffer[bufferStart];
  u.b[1] = buffer[bufferStart + 1];
  u.b[2] = buffer[bufferStart + 2];
  u.b[3] = buffer[bufferStart + 3];

  return u.fval;

}


float convert4BytesToAM2315Float(byte *buffer, int bufferStart)
{


  union u_tag {
    byte b[4];
    float fval;
  } u;


  u.b[0] = buffer[bufferStart + 3];
  u.b[1] = buffer[bufferStart + 2];
  u.b[2] = buffer[bufferStart + 1];
  u.b[3] = buffer[bufferStart + 0];
  Serial.print("fval=");
  Serial.println(u.fval);

  return u.fval;


}

float convert4BytesToFloat(byte *buffer, int bufferStart)
{


  union u_tag {
    byte b[4];
    float fval;
  } u;


  u.b[0] = buffer[bufferStart + 0];
  u.b[1] = buffer[bufferStart + 1];
  u.b[2] = buffer[bufferStart + 2];
  u.b[3] = buffer[bufferStart + 3];


  return u.fval;








}



int interpretBuffer(byte *buffer, int buflen)
{
  if (!((buffer[0] == 0xAB) && (buffer[1] == 0x66)))
  {
    // start bytes are not in buffer - reject
    return 1; // no start bytes
  }
  Serial.println("Start Bytes Found");

  if (buflen != 63)
  {
    return 2; // buflen wrong
  }
  unsigned short checksumValue;

  // calculate checksum
  checksumValue = crc.XModemCrc(buffer, 0, 59);
  Serial.print("crc = 0x");
  Serial.println(checksumValue, HEX);

  if ((checksumValue >> 8) != buffer[61])
  {
    // bad checksum
    return 3;  // bad checksum

  }
  if ((checksumValue & 0xFF) != buffer[62])
  {
    // bad checksum
    return 3;  // bad checksum

  }



  //

  Serial.println(F("Correct Buffer Length"));

  Serial.print(F("Protocol="));
  Serial.println(buffer[2]);

  Serial.print(F("TimeSinceReboot(msec)="));
  Serial.println(convert4BytesToLong(buffer, 3));

  Serial.print(F("Wind Direction="));
  Serial.println(convert2BytesToInt(buffer, 7));

  Serial.print(F("Average Wind Speed (KPH)="));
  Serial.println(convert4BytesToFloat(buffer, 9));

  Serial.print(F("Wind Clicks="));
  Serial.println(convert4BytesToLong(buffer, 13));

  Serial.print(F("Total Rain Clicks="));
  Serial.println(convert4BytesToLong(buffer, 17));

  Serial.print(F("Max Wind Gust="));
  Serial.println(convert4BytesToFloat(buffer, 21));



  Serial.print(F("Outside Temperature="));
  Serial.println(convert4BytesToFloat(buffer, 25));

  Serial.print(F("OT Hex="));
  Serial.print(buffer[25], HEX);
  Serial.print(buffer[26], HEX);
  Serial.print(buffer[27], HEX);
  Serial.println(buffer[28], HEX);

  Serial.print(F("Outside Humidity="));
  Serial.println(convert4BytesToFloat(buffer, 29));

  Serial.print(F("BatteryVoltage="));
  Serial.println(convert4BytesToFloat(buffer, 33));
  Serial.print(F("BatteryCurrent="));
  Serial.println(convert4BytesToFloat(buffer, 37));
  Serial.print(F("LoadCurrent="));
  Serial.println(convert4BytesToFloat(buffer, 41));
  Serial.print(F("SolarPanelVoltage="));
  Serial.println(convert4BytesToFloat(buffer, 45));
  Serial.print(F("SolarPanelCurrent="));
  Serial.println(convert4BytesToFloat(buffer, 49));

  Serial.print(F("AuxA="));
  Serial.println(convert4BytesToFloat(buffer, 53));

  Serial.print(F("Message ID="));
  Serial.println(convert4BytesToLong(buffer, 57));


  Serial.print(F("Checksum High=0x"));
  Serial.println(buffer[61], HEX);
  Serial.print(F("Checksum Low=0x"));
  Serial.println(buffer[62], HEX);



  return 0;

}

// I2C interface - This device communicates with the master via I2C.

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void I2CrequestEvent() {
  // respond with the lastGoodMessage
  // as expected by master
  //Wire.write(lastGoodMessage,64);
  byte command;
  command = Wire.read();
  Serial.print("Command=");
  Serial.println(command);
  byte myBuffer[32];
  int i;
  if (command == 0)
  {
    for (i = 0; i < 32; i++)
    {
      myBuffer[i] = lastGoodMessage[i];
    }
    Wire.write(myBuffer, 32);
    return;
  }

  if (command == 1)
  {

    for (i = 0; i < 32; i++)
    {
      myBuffer[i] = lastGoodMessage[i + 32];
    }
    Wire.write(myBuffer, 32);
    return;
  }

  if (command == 255)
  {

    Serial.print("Toggle=");
    Serial.println(toggle);
    // toggle between 0 and 1
    if (toggle == 1)
    {
      toggle = 0;
      for (i = 0; i < 32; i++)
      {
        myBuffer[i] = lastGoodMessage[i + 32];
      }
      Wire.write(myBuffer, 32);
      return;

    }

    if (toggle == 0)
    {
      toggle = 1;
      for (i = 0; i < 32; i++)
      {
        myBuffer[i] = lastGoodMessage[i];
      }
      Wire.write(myBuffer, 32);
      return;


    }

  }


}

void I2CreceiveEvent(int count)
{
  Serial.print("RE Count =");
  Serial.println(count);

}

void setup()
{

  // set up I2C at address 0x08
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(I2CrequestEvent); // register event
  Wire.onReceive(I2CreceiveEvent);


  Serial.begin(115200);  // Debugging only
  Serial.println("-------Receive Started---------");
  Serial.print("Software Version:");
  Serial.println(SOFTWAREVERSION);

  if (!rf95.init())
  {
    Serial.println("init failed");
    while (1);
  }



  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  //rf95.setTxPower(13, false);

  rf95.setFrequency(434.0);

  int Bw31_25Cr48Sf512 = 2;

  rf95.setModemConfig(RH_RF95<SoftwareSerial>::ModemConfigChoice(Bw31_25Cr48Sf512));
  //rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);

  //rf95.setTxPower(5);

  //rf95.printRegisters();

  consecutiveGoodMessages = 0;
  int i;
  for (i = 0; i < 64; i++)
  {
    lastGoodMessage[i] = 0;


  }

  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);


}


void blinkGood()
{
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(100);

}
void blinkError()
{
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
}

void loop()
{

  delay(1000);

  //if (rf95.available())           // if date is coming from software serial port
  if (rf95.waitAvailableTimeout(6000))           // if date is coming from software serial port

  {

    //Serial.println("Data is coming in");
    //Serial.print("freeMemory()=");
    //Serial.println(freeMemory());


    //buflen = 63;
    buflen = 75;
    byte messageLength;


    //Serial.print("Available =");
    //Serial.println(rf95.available());

    if (rf95.recv(buffer, &buflen))
    {
      //Serial.println("Message Received");
    }
    else
    {
      //Serial.println("No Message Received");
    }

    messageLength = buflen;   // clear off LoRa corruption byte
    //Serial.print(F("messageLength="));
    //Serial.println(messageLength);

    /* for (int i = 0; i < buflen; i++) {
       Serial.print(" ");
       if (buffer[i] < 16)
       {
         Serial.print(F("0"));
       }
       Serial.print(buffer[i], HEX);           //  write buffer to hardware serial port
      }

      Serial.println();
    */
    //printBuffer( buffer, buflen);


    int interpretResult = interpretBuffer(buffer, buflen);

    switch (interpretResult)
    {
      case 0:
        {
          //Serial.println(F("Good Message"));
          int previousGoodMessageID = lastGoodMessageID;
          goodMessages++;

          lastGoodMessageID = convert4BytesToLong(buffer, 57);

          if (lastGoodMessageID == previousGoodMessageID + 1)
          {
            consecutiveGoodMessages++;
          }
          /*Serial.print(F("Current Message ID="));
            Serial.print(lastGoodMessageID);
            Serial.print(F(" Consecutive Good Messages ="));
            Serial.println(consecutiveGoodMessages);
          */
          digitalWrite(LED, HIGH);
          delay(100);
          digitalWrite(LED, LOW);
          // copy bytes to lastGoodMessage array
          // disable interrupts
          //noInterrupts();
          int i;
          for (i = 0; i < 63; i++)
          {
            lastGoodMessage[i] = buffer[i];

          }
          lastGoodMessage[63] = 0;

          // enable interrupts
          //interrupts();

          blinkGood();

        }
        break;
      case 1:
        //Serial.println(F("Bad Message - No Start Bytes"));
        badMessages++;
        consecutiveGoodMessages = 0;
        resetSoftSerialBuffer();
        blinkError();

        break;
      case 2:
        //Serial.println(F("Bad Message - buffer length incorrect"));
        badMessages++;
        consecutiveGoodMessages = 0;
        resetSoftSerialBuffer();
        blinkError();
        break;
      case 3:
        //Serial.println(F("Bad Message - Bad Checksum"));
        badMessages++;
        consecutiveGoodMessages = 0;
        resetSoftSerialBuffer();
        blinkError();


        break;
      default:

        //Serial.print(F("Bad Message - Unknown Return Code ="));
        Serial.println(interpretResult);
        badMessages++;
        consecutiveGoodMessages = 0;
        resetSoftSerialBuffer();
        blinkError();
        break;
    }
    /*
        Serial.print(F("RSSI: "));
        Serial.println(rf95.lastRssi(), DEC);

    */
    Serial.print("GM: ");
    Serial.print(goodMessages);
    Serial.print(" BM: ");
    Serial.println(badMessages);


    /*
        clearBufferArray(buflen);              // call clearBufferArray function to clear the stored data from the array
    */

    int i;



    buflen = 0;


  }

  delay(100);


}

void clearBufferArray(int buflen)              // function to clear buffer array
{
  for (int i = 0; i < buflen; i++)
  {
    buffer[i] = NULL; // clear all index of array with command NULL
  }
}

void resetSoftSerialBuffer()
{
  /*

    Serial.println("Resetting SoftSerial Buffer");
    // delay 1 second
    delay(1000);

    // clear Softserial buffer

    char discard;

    while (SoftSerial.available() > 0)
    {
      discard = SoftSerial.read();

    }
  */
}
