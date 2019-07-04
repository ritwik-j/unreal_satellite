//This code takes an input from system 2 and displays it on the PC
//It also take input from both system 1 & 2 and displays it on an LCD
//By Ritwik Jha

#include <Wire.h>
#include <Time.h>
#include <TimeLib.h>
#include <Servo.h>
const int interval = 1000;

//setup LCD output
#include <LCDi2cR.h>                      
LCDi2cR lcd = LCDi2cR(4,20,0x63,0);

//setup XBee
#include <XBee.h>
XBee xbee               = XBee();

// create reusable response objects for responses we expect to handle 
XBeeResponse response   = XBeeResponse();
ZBRxResponse        rx  = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
uint8_t payload[4] = {0};

// SH + SL Address of receiving XBee                             // Remote XBee 0x40B9D093 0x40A91AAF   
XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x40B9D093); // address of the receiver XBee
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

//declaring variables to be received

double temperature_converted, pressure_value, altitude, heading, humidity_voltage, sensor_RH, true_RH;

//declaring variables to be transmitted
int keyInput;

unsigned long t, t_key;

//declaring function prototypes 
void transmit(int);
void runsensors();
bool receive();
void conversion(int, long, int, int, int, int);
void joystick();

// Joystick x- and y- axes are connected to A1 and A0 analog pins of Arduino.
// Servos are connectd to PWM Pins 9 and 10.
Servo tilt, pan; 

int joyH = A1; //  X - axis of Joystick
int joyV = A0; // y - axis of Joystick
int h = 1500, v = 1500, h_value = 0, v_value = 0;

void setup() 
{
  Wire.begin(); 
  Serial.begin(115200);
  Serial2.begin(115200);
  xbee.setSerial(Serial2);
  
  // setup output connection to LCD Monitor
  lcd.init();

  tilt.attach(5); // TILT on PIN 5
  pan.attach(3); // PAN on PIN 3

}

void loop() 
{
  int tempReceived;
  long pressure;
  int x, y, z;
  int humidity;
  
  //timeperiod of keypress
  runsensors();
  
  if (receive(&tempReceived, &pressure, &x, &y, &z, &humidity))
  {
      conversion (tempReceived, pressure, x, y, z, humidity);    
      printLCD(temperature_converted, altitude, heading, true_RH);
      Serial.print("Temperature: ");
      Serial.println(temperature_converted);
      Serial.print("Altitude:    ");
      Serial.println(altitude);
      Serial.print("Heading:     ");
      Serial.println(heading);
      Serial.print("Humidity:    ");
      Serial.println(true_RH);
  }

  transmit(keyInput);
  Serial.println("-------------------------------------------------------");
  
  //delay(100);
}

void runsensors()
{
  
  keyInput = lcd.keypad();

}

//code for LCD to print system 2 data
void printLCD (double temp, double alt, double hdng, double humid)
{ lcd.clear();
  //delay(10);
  lcd.setCursor(0,1);
  lcd.print("TMP");
  lcd.setCursor(0,11);
  lcd.print("ALTITUDE");
  lcd.setCursor(1,1);
  lcd.print(temp);
  lcd.setCursor(1,11);
  lcd.print(alt);
  lcd.setCursor(2,1);
  lcd.print("HEADING");
  lcd.setCursor(3,1);
  lcd.print(hdng);
  lcd.setCursor(2,11);
  lcd.print("HUMIDITY");
  lcd.setCursor(3,11);
  lcd.print(humid);
  //delay(100);
}



//function to receive packet from system 2 
bool receive(int* tempReceived, long* pressure, int* x, int* y, int* z, int* humidity) 
{
  xbee.readPacket(100);
  if (xbee.getResponse().isAvailable()) 
  {
    //Serial.println("Available");
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) 
    {
      xbee.getResponse().getZBRxResponse(rx);
      Serial.println("Data Received.");
      *tempReceived = rx.getData(0) << 8 | rx.getData(1);
      *pressure = (long)rx.getData(2) << 24 | (long)rx.getData(3) << 16 | (long)rx.getData(4) << 8 | (long)rx.getData(5);
      *x = rx.getData(6) << 8 | rx.getData(7);
      *y = rx.getData(8) << 8 | rx.getData(9);
      *z = rx.getData(10) << 8 | rx.getData(11);
      *humidity = rx.getData(12) << 8 | rx.getData(13);
      return 1;
    }
    else 
    { 
      Serial.println("Data Failure.");
      return 0;
    }
  }
  else 
  { 
    Serial.println("Data Not Available."); 
    return 0;
  }
  
}

//function to transmit system 1 data to system 2
void transmit (int keyvalue) {

  //payload for key press
  payload[0] = keyvalue >> 8 & 0xff;
  payload[1] = keyvalue & 0xff;

  payload[2] = analogRead(joyH)/4;
  payload[3] = analogRead(joyV)/4;
/*
  Serial.print("JoyH ");
  Serial.println(analogRead(joyH));
  Serial.print("JoyV ");
  Serial.println(analogRead(joyV));
*/
  
  // Actually send
  xbee.send(zbTx);

  // After sending, expect a response, give up to 500 ms for the response.
  if (xbee.readPacket(100))
  {
    // If we get a response, it should be a status response
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE)
    {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      // If the 5th byte (delivery status) is successful, print success
      if (txStatus.getDeliveryStatus() == SUCCESS)
      {
        Serial.println("Transmission successful.");
        Serial.println(".");
        //errorcount = 0;
      }
      else
      {
        // the other XBee did not receive
        Serial.println("Transmission error - Did Not Receive Packet");
        Serial.println(".");
        //errorcount++;
      }
    }
    else
    {
      Serial.println("Transmission error - Response is not Status Response");
    }
  }
  else
  {
    // local XBee did not provide a timely TX Status Response
    Serial.println("Transmission error - Timely TX Status Response");
    Serial.println(".");
    //errorcount++;
  }
  /*xbee.send(zbTx);
  
  if (xbee.readPacket(100)) 
  {
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) 
    {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      Serial.println("Transmission Successful.");   
    }
  } 
  else 
  {
    Serial.println("XBee did not provide a timely TX status response.");
  }*/
}

//function to convert raw data received from system 2 to procesed data 
void conversion (int temperature, long pressure, int x, int y, int z, int humidity) 
{
  temperature_converted = (temperature & 0x2fff) * 0.0625;
  double p_value = pressure;
  altitude = 44330 * (1 - pow((p_value / 101000), (1/5.255)));
  
  heading = atan2(x, y)/0.0174532925;
      
      if (heading < 0) 
      { 
        heading+=360; 
      }
      
      heading = 360 - heading; // N=0/360, E=90, S=180, W=270

  double humid = humidity;
  humidity_voltage = humid / 1023 * 5; // Conversion to voltage, using 5V supply
  sensor_RH = (humidity_voltage - 0.958) / 0.0307;
  true_RH = sensor_RH / (1.0546 - 0.00216 * (temperature_converted)); 
}
