#include <Adafruit_BMP085.h>
#include <XBee.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <TimeLib.h>


XBee xbee = XBee();

// For Transfer
uint8_t payload[14] = {0};
XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x40A91AAF);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

// For Receive
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

// For Servos
/* Servos are connectd to PWM Pins 5 and 3
 *  Pan connect to 3
 *  Tilt connect to 5
 */
Servo tilt, pan, solar_open; 
int h_value = 1500, v_value = 1500, h = 0, v = 0;


// For Barometer
#define BMP085_ADDRESS 0x77  // I2C address of BMP085
double pressure_value;
double altitude;
const unsigned char OSS = 0;  // Oversampling Setting
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
long b5;
short temperature;
long pressure;

// For Temperature Sensor
#define TMP102_I2C_ADDRESS 0x48
void getTemp102();

// For Magnetometer
#define HMC_Address 0x1E
int x, y, z;
double heading;

// For Humidity Sensor
const int HumidityOutPin = A7; // (7) pin for the analogue input
unsigned int humidity_value;
double humidity_voltage, sensor_RH, true_RH;

// For Receiving
int keyinput;
double tempinput;
int errorcount;

// For Card Check Function
int key1 = 0, key2 = 0, key3 = 0, key4 = 0;
int currentmode;


// For SD Card
int sdcount = 0;
int filenum = 0;
char filename[16];
// make it long enough to hold your longest file name, plus a null terminator

const float p0 = 101325;   //Pressure at sea level (Pa)

// Chip Select Pin on Arduino
const int chipSelect = 53;

// For Jumper - Arduino Software Reset
#define JumperPin 7

// Declaration of function
void transmit(short, long, int, int, int, int);
void runsensors();
void receive();
void checkmode();
void bmp085Calibration();
short bmp085GetTemperature(unsigned int ut);
long bmp085GetPressure(unsigned long up);
char bmp085Read(unsigned char address);
int bmp085ReadInt(unsigned char address);
unsigned int bmp085ReadUT();
unsigned long bmp085ReadUP();
void sd_write();
void sd_read();
void sd_read_value(char);
void joystick();
bool solar_read();
void solar();

void setup()
{

  // Reset Setup
  /*digitalWrite(JumperPin, HIGH);
  pinMode(JumperPin, OUTPUT);
  errorcount = 0;*/
  
  // Serial and i2c Setup
  Serial.begin(115200);
  Serial.println("Initializing...");

  Serial2.begin(115200);
  xbee.setSerial(Serial2);
  
  Wire.begin();
  Serial.println("XBee and i2c initialized. Initializing sensors...");

  //Joystick and servo setup
  tilt.attach(5); // TILT on PIN 5
  pan.attach(3); // PAN on PIN 3

  //Servo setup
  solar_open.attach(6); // SOLAR OPEN on PIN 6
  
  // Barometer Setup
  bmp085Calibration();
  Serial.println("Barometer initialized.");

  // Magnetometer Setup
  Wire.beginTransmission(HMC_Address);
  Wire.write((byte)0x02); // select mode register
  Wire.write((byte)0x00); // continuous measurement mode
  Wire.endTransmission();
  Serial.println("Magnetometer initialized. Starting loop sequence...");

  if (currentmode != 0)
  {
    Serial.println("Current mode not set to Normal Mode. Setting to Normal...");
    currentmode = 0;
  }

  //SD Card Initialisation
  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect))
  {
    Serial.println("Initalization failed, or SD card not present.");
    // Initialisation failed, do not continue
    return;
  }
  // Initialisation Complete
  Serial.println("Card initialized.");

  //For SD Card Reset
  int n = 0;
  snprintf(filename, sizeof(filename), "DATA%04d.txt", n);  
  Serial.println("Clearing SD Card...");
  // includes a three-digit sequence number in the file name
  while(SD.exists(filename)) 
  {

    SD.remove(filename);
    if (SD.exists(filename))
    {
      Serial.print("Could not delete ");
      Serial.print(filename);
    }
    n++;
    snprintf(filename, sizeof(filename), "DATA%04d.txt", n);
  }
  Serial.println("SD Card cleared.");
  

}

void loop()
{

  if (currentmode == 0)
  {
    runsensors();
    sd_write();
  } else if (currentmode == 1)
  {
    sd_read();
  }

  // Short delay to ensure syncing between boards
  //delay(100);

  // Initiate transmission of system 1 sensor data
  transmit(temperature, pressure, x, y, z, humidity_value);

  // Wait up to 500ms for a transmission to be received
  if (xbee.readPacket(200))
  {
    // If transmission was received, scan the data and print it.
    receive();
    if (keyinput != -16)
    {
      checkmode();
    }
  }
  else
  {
    // Transmission was not received from other XBee
    Serial.println("Error - No Transmission Received");
    Serial.println(".");
    errorcount++;
  }

  //Deploying solar panel
  /*if(solar_read){ //check for key input "#*#"
    solar();      //deploy solar panel
  }*/
  
  /*while (errorcount >= 20)
  {
    Serial.println("High Consecutive Packet Loss, Resetting....");
    Serial.println("----------------------------------------------------------------");
    //digitalWrite(JumperPin, LOW);
    Serial.println("Reset failed, retrying.");
    //delay(10);
  }*/
  
  // Delay to ensure re-syncing and readibility of data
  // delay(300);
}

// Function to gather all sensor data
void runsensors()
{
  // Barometer Running
  // temperature = bmp085GetTemperature(bmp085ReadUT()); This is no longer needed as we take temperature from TMP102.
  pressure = bmp085GetPressure(bmp085ReadUP());

  /* Calculation of Altitude, not needed
    pressure_value = (double)pressure;
    altitude = 44330 * (1 - pow((pressure_value / 101000), (1 / 5.255))); */

  // Temperature Sensor Running
  getTemp102();

  // Magnetometer Running
  Wire.beginTransmission(HMC_Address); //select register 3, X MSB register
  Wire.write((byte)0x03);
  Wire.endTransmission(); // Read data from each axis, 2 registers per axis
  Wire.requestFrom(HMC_Address, 6);

  if ( 6 <= Wire.available() )
  {
    x = Wire.read() << 8; // X msb
    x |= Wire.read(); // X lsb
    z = Wire.read() << 8; // Z msb
    z |= Wire.read(); // Z lsb
    y = Wire.read() << 8; // Y msb
    y |= Wire.read(); // Y lsb

    /* Calculation of Heading, not needed
      heading = atan2(x, y) / 0.0174532925;

      if (heading < 0)
      {
      heading += 360;
      }
      heading = 360 - heading; // N=0/360, E=90, S=180, W=270
      }
      else
      {
      x = 0;
      y = 0;
      z = 0; */
  }


  // Humidity Sensor Running
  humidity_value = 0;
  humidity_value = analogRead(HumidityOutPin); // Pin A7 in this case

  /* Further Humidity Calculations, not needed
    humidity_voltage = (double)humidity_value / 1023 * 5; // Conversion to voltage, using 5V supply
    sensor_RH = (humidity_voltage - 0.958) / 0.0307;
    true_RH = sensor_RH / (1.0546 - 0.00216 * (temperature / 10)); */
}

// Function for transmission of data
void transmit(short temperature, long pressure, int x, int y, int z, int humidity_value)
{
  // Setting up of transmission payloads
  payload[0] = temperature >> 8 & 0xff;
  payload[1] = temperature & 0xff;

  payload[2] = pressure >> 24 & 0xff;
  payload[3] = pressure >> 16 & 0xff;
  payload[4] = pressure >> 8 & 0xff;
  payload[5] = pressure & 0xff;

  payload[6] = x >> 8 & 0xff;
  payload[7] = x & 0xff;

  payload[8] = y >> 8 & 0xff;
  payload[9] = y & 0xff;

  payload[10] = z >> 8 & 0xff;
  payload[11] = z & 0xff;

  payload[12] = humidity_value >> 8 & 0xff;
  payload[13] = humidity_value & 0xff;

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
}

// Function to receive data from other XBee
void receive()
{
  // Read the received transmission
  xbee.readPacket(100);

  //Serial.println("Data received.");
  xbee.getResponse().getZBRxResponse(rx);

  // Scan and convert the data, then print
  int kvalue = rx.getData(1);
  keyinput = kvalue - 48;

  Serial.print("Currently Pressed Key: ");
  if (keyinput >= 0)
  {
    Serial.println(keyinput);
  }
  else if (keyinput == -6)
  {
    Serial.println("*");
  }
  else if (keyinput == -13)
  {
    Serial.println("#");
  }
  else if (keyinput == -16)
  {
    Serial.println("None");
  }
  else
  {
    Serial.println("Keypad Error");
    errorcount++;
  }

  Serial.println("-------------------------------------");


  //read joystick input
  h = rx.getData(2);
  v = rx.getData(3);
  joystick();
    
}

// Function to check current mode and last 3 pressed keys, switch if string is correct
void checkmode()
{

  key1 = key2;
  key2 = key3;
  key3 = key4;
  key4 = keyinput;

  if (currentmode == 1)
  {
    if ((key2 == -6) && (key3 == -6) && (key4 == -6))
    {
      currentmode = 0;
      Serial.println("Mode switched to Normal Mode.");
    } 
    if ((key2 == 0) && (key3 == 0) && (key4 == 0))    
    {
      Serial.println("Enter 4 digit input:"); ///////////////////////////////////////////////////////////////
    } 
  } 
  else if ((key2 == -13) && (key3 == -13) && (key4 == -13))
  {
    currentmode = 1;
    Serial.println("Mode switched to Replay Mode. Reading latest file.");
  } 
}

// Temperature Sensor Run
void getTemp102()
{
  // bytes used to store value from the TMP102 temperature registers
  byte firstByte, secondByte;

  Wire.beginTransmission(TMP102_I2C_ADDRESS);
  Wire.write((byte)0x00);  // point to temperature register
  Wire.endTransmission();

  Wire.requestFrom(TMP102_I2C_ADDRESS, 2);
  if ( Wire.available() >= 2 )
  {
    // read the TMP102 temperature register
    firstByte  = (Wire.read());
    secondByte = (Wire.read());
  }

  // MSB
  temperature = (firstByte << 4);
  //LSB
  temperature |= (secondByte >> 4);
}

// Functions for Barometer
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  if (ut == 0)
  {
    return 0;
  }
  long x1, x2;

  x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8) >> 4);
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  if (up == 0)
  {
    return 0;
  }
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6) >> 12) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;

  // Calculate B4
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

  b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write((byte)address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  for (int i = 0; i <= 100; i++) {
    if (Wire.available()) {
      return Wire.read();
    }
    delay(1);
  }
  return 0x00;
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write((byte)address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  for (int i = 0; i <= 100; i++) {
    if (Wire.available() >= 2) {
      msb = Wire.read();
      lsb = Wire.read();
      return (int) msb << 8 | lsb;
    }
    delay(1);
  }
  return 0;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write((byte)0xF4);
  Wire.write((byte)0x2E);
  Wire.endTransmission();

  // Wait at least 5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write((byte)0xF4);
  Wire.write((byte)(0x34 + (OSS << 6)));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3 << OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write((byte)0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);

  // Wait for data to become available
  for (int i = 0; i <= 100; i++) {
    if (Wire.available() >= 3) {
      msb  = Wire.read();
      lsb  = Wire.read();
      xlsb = Wire.read();

      up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8 - OSS);

      return up;
    }
    delay(1);
  }
  return 0;
}
// Functions for Barometer END

//Functions for Joystick BEGIN
void joystick () {

  int hor, ver; 
  
  hor = map(h, 0, 255, -50, 50);
  ver = map(v, 0, 255, -50, 50);
  /*
  Serial.print("hor=");
  Serial.println(hor);
  Serial.print("ver=");
  Serial.println(ver);
  */

  if ((h_value < 2040) && (h_value > 970))
    {
      if ((h_value + hor < 1900) || (h_value + hor > 1050)){
        h_value += hor;
      }
    }
  if ((v_value < 2040) && (v_value > 970)){
    if ((v_value + ver < 1900) || (v_value + ver > 1050)){
        v_value += ver;
      }
  }
  
  Serial.print("h_value ");
  Serial.println(h_value);
  Serial.print("v_value ");
  Serial.println(v_value);

  tilt.write(h_value);
  pan.write(v_value);
  delay(15);
}

// Fuctions for SD Card BEGIN
void sd_write() {

  unsigned int temp_store;
  unsigned long pressure_store;
  unsigned int xs, ys, zs;

  snprintf(filename, sizeof(filename), "data%04d.txt", filenum); 
  // includes a three-digit sequence number in the file name
  while(SD.exists(filename)) {
    filenum++;
    snprintf(filename, sizeof(filename), "data%04d.txt", filenum); 
    if (filenum >= 10000)
    {
      Serial.println("10,000 file limit reached. Overriding last file data...");
      filenum--;
      snprintf(filename, sizeof(filename), "DATA%04d.txt", filenum);
      SD.remove(filename);
    }
  }
  
  Serial.print("Writing to: ");
  Serial.println(filename);
  // Now filename[] contains the name of a file that doesn't exist

  // Open file for writing
  File dataFile = SD.open(filename, FILE_WRITE);

  // Check if file is available
  if (dataFile) {

    // File Available
    // Create temporary variables
    temp_store = temperature;
    pressure_store = pressure;
    xs = x;
    ys = y;
    zs = z;

    // Read and Store Sensor Data to SD Card
    dataFile.print(temp_store);
    dataFile.print(",");
    dataFile.print(pressure_store);
    dataFile.print(",");
    dataFile.print(xs);
    dataFile.print(",");
    dataFile.print(ys);
    dataFile.print(",");
    dataFile.print(zs);
    dataFile.print(",");
    dataFile.print(humidity_value);
    dataFile.print(",");

    // Notify on monitor
    Serial.println("Data logged in SD Card.");

    // Writing done, close file
    dataFile.close();

  }

  // If the file isn't open, show error
  else {
    Serial.print("Error - ");
    Serial.print(filename);
    Serial.print(" not opened");
  }
}

void sd_read()
{
  if ((keyinput >= 0) && (key1 >= 0) && (key2 >= 0) && (key3 >= 0) && (key4 >= 0))
  {
    sdcount = (key1 * 1000) + (key2 * 100) + (key3 * 10) + key4; // Reads the last 4 keypresses /////////////////////////////////////////////
    snprintf(filename, sizeof(filename), "data%04d.txt", sdcount);  
  }

  // If file exists, read it, if not, default back to normal mode to prevent crash.
  if (SD.exists(filename))
  {
    File dataFile = SD.open(filename, FILE_READ);

    Serial.print("Currently Reading ");
    Serial.println(filename);

    temperature = sd_read_value(dataFile);
    pressure = sd_read_value(dataFile);
    x = sd_read_value(dataFile);
    y = sd_read_value(dataFile);
    z = sd_read_value(dataFile);
    humidity_value = sd_read_value(dataFile);

    dataFile.close();

    Serial.println("Data read from SD Card.");
  } 
  else
  {
    Serial.print("File ");
    Serial.print(filename);
    Serial.println(" does not exist. Reverting to normal mode and resuming sensors.");
    currentmode = 0;
    runsensors();
  }
}

unsigned long sd_read_value(File dataFile)
{
  unsigned long data = 0;
  while (dataFile.available())
  {
    char value = dataFile.read();
    if (value == ',')
    {
      break;
    }
    else
    {
      data *= 10;
      data += (int)value - 48;
    }
  }
  return data;
}
// Functions for SD Card END

//Function for solar read BEGIN
bool solar_read(){

  if ((keyinput >= 0) && (key1 == -13) && (key2 == -6) && (key3 == -13))
    {
      return 1;
     }
  
  return 0;
}
//Function for solar read END

//Function for solar panel to open BEGIN
void solar(){

  solar_open.write(2100);

  Serial.println("Solar panel has been deployed");

}
//Function for solar panel to open END
