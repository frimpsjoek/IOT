#include <WiFi.h> //imports wifi library 
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h" //imports library for asynchronous webserver
#include "Wire.h"    // imports the wire library for talking over I2C 
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"  // import the Pressure Sensor Library We are using Version one of Adafruit API for this sensor
Adafruit_BME680 mySensor; // create sensor object called mySensor  
#include <SPI.h> //imports SPI library for SD card interface
#include <TimeLib.h> //import time library
#include <SdFat.h> // sdcard library for sdmmc 
#include <WiFiUdp.h>


SdFat sd; //creates sdcard class as sd
SdFile myFile; //creates sdcard file
char newfile[] = "BME680test.csv";   //create name of csv file

//definition of pins for sdcard and LED pin
#define LED 15
#define SCK 18
#define MISO 19
#define MOSI 23
#define CS 5
#define SEALEVELPRESSURE_HPA (1013.25)

//definition of local WIFI 
const char* ssid = "NETGEAR91";
const char* password = "silkyflute122";

//function for WIFI status
String get_wifi_status(int status){
    switch(status){
        case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
        case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
        case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
        case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
        case WL_CONNECTED:
        return "WL_CONNECTED";
        case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    }
}


//create variables for the sensors
float temperature;
float pressure;
float humidity;
float altitude;
float gas_res;
float lastWriteTime;


// NTP Servers to retrieve the time:
IPAddress timeServer(195, 186, 4, 101); // 195.186.4.101 (bwntp2.bluewin.ch)
const char* ntpServerName = "ch.pool.ntp.org";
const int timeZone = -3;     // Central European Time (summer time)

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
AsyncWebServer server(80);  //AsyncWebServer at port 80             
           


//////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(115200); //turn on serial monitor
//mySensor.begin(0x76);   //initialize pressure sensor mySensor

 int status = WL_IDLE_STATUS;
  Serial.println("\nConnecting");
  Serial.println(get_wifi_status(status));
  WiFi.begin(ssid, password);
  while(status != WL_CONNECTED){
      delay(500);
      status = WiFi.status();
      Serial.println(get_wifi_status(status));
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");

  Serial.print("Using NTP Server ");
  Serial.println(ntpServerName);
  
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServer);
  Serial.print("NTP Server IP ");
  Serial.println(timeServer);
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(60);

  pinMode(LED,OUTPUT); //set LED as an output


  while (!Serial);
  Serial.println(F("BME680 async test"));
  if (!mySensor.begin()) {
  Serial.println(F("Could not find a valid BME680 sensor, check wiring!")); //check to figure out whether the BME sensor works
  while (1);
    }

  // Set up oversampling and filter initialization
  mySensor.setTemperatureOversampling(BME680_OS_8X);
  mySensor.setHumidityOversampling(BME680_OS_2X);
  mySensor.setPressureOversampling(BME680_OS_4X);
  mySensor.setIIRFilterSize(BME680_FILTER_SIZE_3);
  mySensor.setGasHeater(320, 150); // 320*C for 150 ms



  while (!Serial) {}  // wait for Serial
  Serial.println("Type any character to start");
  while (Serial.read() <= 0) {}
  delay(400);  // catch Due reset problem

  while (!sd.begin(CS, SPI_HALF_SPEED)) {}
  myFile.open(newfile, O_WRITE | O_CREAT | O_APPEND);  // open the file for write at end like the Native SD library


  if (myFile.size() == 0) {
    myFile.println("timestamp, temperature, pressure, humidity, gas res, altitude ");
    Serial.println("Header line added to the file.");
    }

  myFile.close();
  Serial.println("file writeup setup is complete!"); //would print to the Arduino screen if connected
  delay(50); 

}



/*-----------------------------NTP code --------------------------------------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();


//asynchronous website generator with the sensor readings as a dashboard. ptr represents the html code that generates the website with css styling for visual effect
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  temperature = mySensor.readTemperature();
  humidity = mySensor.readHumidity();
  pressure = mySensor.readPressure() / 100.0F;
  altitude = mySensor.readAltitude(SEALEVELPRESSURE_HPA);

  String ptr = "<!DOCTYPE html>";
  ptr +="<html>";
  ptr +="<head>";
  ptr +="<title>ESP32 Weather Station</title>";
  ptr +="<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  ptr +="<link href='https://fonts.googleapis.com/css?family=Open+Sans:300,400,600' rel='stylesheet'>";
  ptr +="<style>";
  ptr +="html { font-family: 'Open Sans', sans-serif; display: block; margin: 0px auto; text-align: center;color: #444444;}";
  ptr +="body{margin: 0px;} ";
  ptr +="h1 {margin: 50px auto 30px;} ";
  ptr +=".side-by-side{display: table-cell;vertical-align: middle;position: relative;}";
  ptr +=".text{font-weight: 600;font-size: 19px;width: 200px;}";
  ptr +=".reading{font-weight: 300;font-size: 50px;padding-right: 25px;}";
  ptr +=".temperature .reading{color: #F29C1F;}";
  ptr +=".humidity .reading{color: #3B97D3;}";
  ptr +=".pressure .reading{color: #26B99A;}";
  ptr +=".altitude .reading{color: #955BA5;}";
  ptr +=".superscript{font-size: 17px;font-weight: 600;position: absolute;top: 10px;}";
  ptr +=".data{padding: 10px;}";
  ptr +=".container{display: table;margin: 0 auto;}";
  ptr +=".icon{width:65px}";
  ptr +="</style>";
  ptr +="</head>";
  ptr +="<body>";
  ptr +="<h1>ESP32 Weather Station</h1>";
  ptr +="<div class='container'>";
  ptr +="<div class='data temperature'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg enable-background='new 0 0 19.438 54.003'height=54.003px id=Layer_1 version=1.1 viewBox='0 0 19.438 54.003'width=19.438px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M11.976,8.82v-2h4.084V6.063C16.06,2.715,13.345,0,9.996,0H9.313C5.965,0,3.252,2.715,3.252,6.063v30.982";
  ptr +="C1.261,38.825,0,41.403,0,44.286c0,5.367,4.351,9.718,9.719,9.718c5.368,0,9.719-4.351,9.719-9.718";
  ptr +="c0-2.943-1.312-5.574-3.378-7.355V18.436h-3.914v-2h3.914v-2.808h-4.084v-2h4.084V8.82H11.976z M15.302,44.833";
  ptr +="c0,3.083-2.5,5.583-5.583,5.583s-5.583-2.5-5.583-5.583c0-2.279,1.368-4.236,3.326-5.104V24.257C7.462,23.01,8.472,22,9.719,22";
  ptr +="s2.257,1.01,2.257,2.257V39.73C13.934,40.597,15.302,42.554,15.302,44.833z'fill=#F29C21 /></g></svg>";
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>Temperature</div>";
  ptr +="<div class='side-by-side reading'>";
  ptr +=(int)temperature;
  ptr +="<span class='superscript'>&deg;C</span></div>";
  ptr +="</div>";
  ptr +="<div class='data humidity'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg enable-background='new 0 0 29.235 40.64'height=40.64px id=Layer_1 version=1.1 viewBox='0 0 29.235 40.64'width=29.235px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><path d='M14.618,0C14.618,0,0,17.95,0,26.022C0,34.096,6.544,40.64,14.618,40.64s14.617-6.544,14.617-14.617";
  ptr +="C29.235,17.95,14.618,0,14.618,0z M13.667,37.135c-5.604,0-10.162-4.56-10.162-10.162c0-0.787,0.638-1.426,1.426-1.426";
  ptr +="c0.787,0,1.425,0.639,1.425,1.426c0,4.031,3.28,7.312,7.311,7.312c0.787,0,1.425,0.638,1.425,1.425";
  ptr +="C15.093,36.497,14.455,37.135,13.667,37.135z'fill=#3C97D3 /></svg>";
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>Humidity</div>";
  ptr +="<div class='side-by-side reading'>";
  ptr +=(int)humidity;
  ptr +="<span class='superscript'>%</span></div>";
  ptr +="</div>";
  ptr +="<div class='data pressure'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg enable-background='new 0 0 40.542 40.541'height=40.541px id=Layer_1 version=1.1 viewBox='0 0 40.542 40.541'width=40.542px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M34.313,20.271c0-0.552,0.447-1,1-1h5.178c-0.236-4.841-2.163-9.228-5.214-12.593l-3.425,3.424";
  ptr +="c-0.195,0.195-0.451,0.293-0.707,0.293s-0.512-0.098-0.707-0.293c-0.391-0.391-0.391-1.023,0-1.414l3.425-3.424";
  ptr +="c-3.375-3.059-7.776-4.987-12.634-5.215c0.015,0.067,0.041,0.13,0.041,0.202v4.687c0,0.552-0.447,1-1,1s-1-0.448-1-1V0.25";
  ptr +="c0-0.071,0.026-0.134,0.041-0.202C14.39,0.279,9.936,2.256,6.544,5.385l3.576,3.577c0.391,0.391,0.391,1.024,0,1.414";
  ptr +="c-0.195,0.195-0.451,0.293-0.707,0.293s-0.512-0.098-0.707-0.293L5.142,6.812c-2.98,3.348-4.858,7.682-5.092,12.459h4.804";
  ptr +="c0.552,0,1,0.448,1,1s-0.448,1-1,1H0.05c0.525,10.728,9.362,19.271,20.22,19.271c10.857,0,19.696-8.543,20.22-19.271h-5.178";
  ptr +="C34.76,21.271,34.313,20.823,34.313,20.271z M23.084,22.037c-0.559,1.561-2.274,2.372-3.833,1.814";
  ptr +="c-1.561-0.557-2.373-2.272-1.815-3.833c0.372-1.041,1.263-1.737,2.277-1.928L25.2,7.202L22.497,19.05";
  ptr +="C23.196,19.843,23.464,20.973,23.084,22.037z'fill=#26B999 /></g></svg>";
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>Pressure</div>";
  ptr +="<div class='side-by-side reading'>";
  ptr +=(int)pressure;
  ptr +="<span class='superscript'>hPa</span></div>";
  ptr +="</div>";
  ptr +="<div class='data AQI Score'>";
  ptr +="<div class='side-by-side icon'>";
  ptr +="<svg enable-background='new 0 0 58.422 40.639'height=40.639px id=Layer_1 version=1.1 viewBox='0 0 58.422 40.639'width=58.422px x=0px xml:space=preserve xmlns=http://www.w3.org/2000/svg xmlns:xlink=http://www.w3.org/1999/xlink y=0px><g><path d='M58.203,37.754l0.007-0.004L42.09,9.935l-0.001,0.001c-0.356-0.543-0.969-0.902-1.667-0.902";
  ptr +="c-0.655,0-1.231,0.32-1.595,0.808l-0.011-0.007l-0.039,0.067c-0.021,0.03-0.035,0.063-0.054,0.094L22.78,37.692l0.008,0.004";
  ptr +="c-0.149,0.28-0.242,0.594-0.242,0.934c0,1.102,0.894,1.995,1.994,1.995v0.015h31.888c1.101,0,1.994-0.893,1.994-1.994";
  ptr +="C58.422,38.323,58.339,38.024,58.203,37.754z'fill=#955BA5 /><path d='M19.704,38.674l-0.013-0.004l13.544-23.522L25.13,1.156l-0.002,0.001C24.671,0.459,23.885,0,22.985,0";
  ptr +="c-0.84,0-1.582,0.41-2.051,1.038l-0.016-0.01L20.87,1.114c-0.025,0.039-0.046,0.082-0.068,0.124L0.299,36.851l0.013,0.004";
  ptr +="C0.117,37.215,0,37.62,0,38.059c0,1.412,1.147,2.565,2.565,2.565v0.015h16.989c-0.091-0.256-0.149-0.526-0.149-0.813";
  ptr +="C19.405,39.407,19.518,39.019,19.704,38.674z'fill=#955BA5 /></g></svg>";
  ptr +="</div>";
  ptr +="<div class='side-by-side text'>AQI Score</div>";
  ptr +="<div class='side-by-side reading'>";
  ptr +=(int)calculateAirQualityScore();
  ptr +="<span class='superscript'>%</span></div>";
  ptr +="</div>";
  ptr +="</div>";
  ptr +="</body>";
  ptr +="</html>";

  request->send(200, "text/html", ptr);
  }); 

  server.begin(); // begin server for the dashboard
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
 
  unsigned long endTime = mySensor.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
}
  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);
   
  if (!mySensor.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }


/*
reads the temperature, pressure, humidity, gas_resistance, altitude and prints it to the screen
*/
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.print(F("Reading completed at "));
  Serial.println(millis());


  Serial.print(F("Temperature = "));
  temperature = mySensor.temperature;
  Serial.print(temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  pressure = mySensor.pressure / 100.0;
  Serial.print(pressure);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  humidity = mySensor.humidity;
  Serial.print(humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  gas_res = mySensor.gas_resistance / 1000.0;
  Serial.print(gas_res);
  Serial.println(F(" KOhms"));

  Serial.print(F("Approx. Altitude = "));
  altitude  = mySensor.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print(altitude);
  Serial.println(F(" m"));

  Serial.print(F("AQI = "));
  Serial.print(calculateAirQualityScore());
  Serial.println(F(" %"));

  Serial.println();

  //prepare data in a dataString to be written to SD
      String dataString = "";
      dataString += temperature;
      dataString += ",";
      dataString += pressure;
      dataString += ",";
      dataString += humidity;
      dataString += ",";
      dataString += gas_res;
      dataString += ",";
      dataString += altitude;



    if (millis() - lastWriteTime >= 10000) {
        lastWriteTime = millis();

while (!sd.begin(CS,SPI_HALF_SPEED)) {  
        }
        
       myFile.open(newfile, O_WRITE | O_APPEND); //Opens the file
       delay(5);
       Serial.print("Writing to csv...");
       //blink while writing data
       
       digitalWrite(LED,HIGH);
       delay(500);
       myFile.print(year());
       myFile.print("-");
       myFile.print(month());
       myFile.print("-");
       myFile.print(day());
       myFile.print(" ");
       myFile.print(hour());
       myFile.print(":");
       myFile.print(minute());
       myFile.print(":");
       myFile.print(second());
       myFile.print(",");
       myFile.print(dataString); //prints data string to the file
       myFile.println("");
       delay(5);
       myFile.close(); //closes the file
       digitalWrite(LED,LOW);
       delay(500); //Pause between readings.

    }

  // re-open the file for reading:
  if (!myFile.open(newfile, O_READ)) {
    sd.errorHalt("opening file for read failed");
  }
  Serial.println("file:");

  // read from the file until there's nothing else in it:
  int data;
  while ((data = myFile.read()) >= 0) Serial.write(data);
  // close the file:
  myFile.close();


}


///////////////////////////////////////////////////////////////////////////////
//function created to display the time and date
void digitalClockDisplay()  {

Serial.print(year());
Serial.print("-");
Serial.print(month());
Serial.print("-");
Serial.print(day());
//Serial.print(" ");
Serial.print(",");
Serial.print(hour());
Serial.print(":");
Serial.print(minute());
Serial.print(":");
Serial.print(second());
Serial.println(" ");
}



//creates the calculateAirQualityScore function
  float calculateAirQualityScore() {
  float hum_weighting = 0.25;
  float gas_baseline = 100000;
  float hum_baseline = 40.0;

  float gas = mySensor.gas_resistance ;
  float gas_offset = gas_baseline - gas;

  float hum = mySensor.humidity;
  float hum_offset = hum - hum_baseline;

  float hum_score;
  float gas_score;

  if (hum_offset > 0) {
    hum_score = (100 - hum_baseline - hum_offset) / (100 - hum_baseline) * (hum_weighting * 100);
  } else {
    hum_score = (hum_baseline + hum_offset) / hum_baseline * (hum_weighting * 100);
  }

  if (gas_offset > 0) {
    gas_score = (gas / gas_baseline) * (100 - (hum_weighting * 100));
  } else {
    gas_score = 100 - (hum_weighting * 100);
  }
  return hum_score + gas_score;
}
