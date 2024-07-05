#include <LiquidCrystal.h>
#include <SoftwareSerial.h> // Library for using serial communication
#include <TinyGPSPlus.h>



const char *gpsStream =
"$GPGGA,163326.228,2630.989,N,08013.930,E,1,12,1.0,0.0,M,0.0,M,,*6A"
"$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30"
"$GPRMC,163326.228,A,2630.989,N,08013.930,E,,,110424,000.0,W*7C";

  
String SMS;
double SPEED = 0.0; 
TinyGPSPlus gps;
SoftwareSerial SIM900(9, 10); // Pins 9, 10 are used as used as software serial pins

String incomingData;   // for storing incoming serial data
//String message = "";   // A String for storing the message
//int relay_pin = 2;    // Initialized a pin for relay module
char msg;
char call;
char c; 
// Initialize the LCD object
LiquidCrystal lcd(2, 3, 4, 5, 6, 7); // (RS, E, D4, D5, D6, D7)
#define buzzer 8

#include "mpu9250.h"

/* Mpu9250 object */
bfs::Mpu9250 imu;

byte updateflag; 

double xaxis = 0, yaxis = 0, zaxis = 0;
double deltx = 0, delty = 0, deltz = 0;
double vibration = 2, devibrate = 75;
double magnitude = 0;
double sensitivity = 15;
double angle;
boolean impact_detected = false;

unsigned long time1;

void setup() {
  // Initialize the LCD with 16 columns and 2 rows
  Serial.begin(115200);
  SIM900.begin(115200); // baudrate for GSM shield
  lcd.begin(16, 2);  
  pinMode(buzzer, OUTPUT); 
  Wire.begin();
  Wire.setClock(400000);
//  delay(1000); 
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  while(!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
  
  delay(100);
  
  // set SMS mode to text mode
  SIM900.print("AT+CMGF=1\r");  
  delay(100);
  
  // set gsm module to tp show the output on serial out
  SIM900.print("AT+CNMI=2,2,0,0,0\r"); 
  delay(100);
  
  while (*gpsStream)
    if (gps.encode(*gpsStream++)){}
}

void loop() {
//  tone(buzzer, 1000); 
//  delay(1000); 
//  noTone(buzzer); 
  
  if (micros() - time1 > 999) Impact();
  
  if(updateflag==1)
  {
    //Crash hua hai 
    lcd.setCursor(0, 0);//(col, row)
    lcd.print("Crash detected");
    lcd.setCursor(0, 1);//(col, row)
    lcd.print("Sending SMS...");
    tone(buzzer, 1000); 
    SendMessage(); 
    delay(1000);
    while(1){
      }
    
  }
  else
  {

    lcd.setCursor(0, 0);//(col, row)
    lcd.print("No crash");
    
  }
  delay(1000);
}

void SendMessage()
{
  SMS = "caution: Tlit sensor is activated !";  
  SMS = SMS + "\n";
  SMS = SMS + "Latitude: ";
  SMS = SMS + gps.location.lat();
  SMS = SMS + "\n";
  SMS = SMS + "Longitude: ";
  SMS = SMS + gps.location.lng();
  SMS = SMS + "\n";
  SMS = SMS + "Speed: ";
  SMS = SMS + (String)SPEED;
  SMS = SMS + "\n";
  SMS = SMS + "https://www.google.com/maps/search/?api=1&query=";
  SMS = SMS + gps.location.lat();
  SMS = SMS + ",";
  SMS = SMS + gps.location.lng();
  SIM900.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  SIM900.println("AT+CMGS=\"+918303019841\"\r"); // Replace x with mobile number
  delay(1000);
  
  SIM900.println("A crash has been detected at the following location\nhttps://maps.app.goo.gl/61Kj5CcwProJpXx89");
  delay(100);
   SIM900.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
  Serial.println("SMS has been sent!");
  lcd.setCursor(0, 1);//(col, row)
  lcd.print("SMS Sent       "); 
  noTone(buzzer); 
}

void Impact()
{
  Serial.println("Impact called"); 
  //--------------------------------------------------------------
  time1 = micros(); // resets time value
  //--------------------------------------------------------------
  double oldx = xaxis; //store previous axis readings for comparison
  double oldy = yaxis;
  double oldz = zaxis;
  while(imu.Read()){
  xaxis = imu.accel_x_mps2();
  yaxis = imu.accel_y_mps2();
  zaxis = imu.accel_z_mps2();
  if(xaxis!=oldx) break; 
  }
  //--------------------------------------------------------------
  //loop counter prevents false triggering. Vibration resets if there is an impact. Don't detect new changes until that "time" has passed.
  vibration--; 
  //Serial.print("Vibration = "); Serial.println(vibration);
  if(vibration < 0) vibration = 0;                                
  //Serial.println("Vibration Reset!");
  
  if(vibration > 0) return;
  //--------------------------------------------------------------
  deltx = xaxis - oldx;                                           
  delty = yaxis - oldy;
  deltz = zaxis - oldz;
  
  //Magnitude to calculate force of impact.
  magnitude = sqrt(sq(deltx) + sq(delty) + sq(deltz));
  if (magnitude >= sensitivity) //impact detected
  {
    Serial.print("Crash Detected. Intensity = "); 
    Serial.println(magnitude); 
    updateflag=1;
    // reset anti-vibration counter
    vibration = devibrate;
  }
  else
  {
    Serial.print("No crash. Magnitude is "); 
    Serial.println(magnitude);
    magnitude=0;
  }
}
