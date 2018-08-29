/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  Attention! Please check out TinyGSM guide:
    http://tiny.cc/tiny-gsm-readme

  WARNING: GSM modem support is for BETA testing.

  You’ll need:
   - Blynk App (download from AppStore or Google Play)
   - Arduino Mega 2560 board
   - Decide how to connect to Blynk
     (USB, Ethernet, Wi-Fi, Bluetooth, ...)

  There is a bunch of great example sketches included to show you how to get
  started. Think of them as LEGO bricks  and combine them as you wish.
  For example, take the Ethernet Shield sketch and combine it with the
  Servo example, or choose a USB sketch and add a code from SendData
  example.
 *************************************************************/
/*

  Decimal: 6148364 (24Bit) Binary: 010111011101000100001100 Tri-State: FF1F1F0F0010 PulseLength: 435   ::: Yellow
  Decimal: 6148355 (24Bit) Binary: 010111011101000100000011 Tri-State: FF1F1F0F0001 PulseLength: 436   ::: Orange
  Decimal: 6148400 (24Bit) Binary: 010111011101000100110000 Tri-State: FF1F1F0F0100 PulseLength: 437   ::: Gray
  Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
#define TINY_GSM_MODEM_SIM808
#define SerialAT Serial1


// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
//#define BLYNK_HEARTBEAT 30
#include <TinyGsmClient.h>
#include <BlynkSimpleSIM800.h>
#include <TimeLib.h>

//Time:
int csq;
int y, m, d, h, mm, s;
int trackingInterval;

//GPS
bool atHome = true;
#define HOME_CORDS_LAT 43.58923549984751 * M_PI / 180.0
#define HOME_CORDS_LON 1.295891482209072 * M_PI / 180.0
double  prevLat;
double  prevLon;
int indexMap = 0;
#define distance_between_two_wayPoint 100 //Unit meter
#define earthRadiusKm 6371000.0

double gps_latitude;
double gps_longitude;
double gps_speed;
int gps_altitude;
int gps_view_satellites;
int gps_used_satellites;
bool gps_fixstatus = false;
bool ghost = false;

//Relays
#define contact 22
#define starter 23
#define signes 24
#define gateController 25
#define relay_x 26
#define doorslock 27
#define back 28
#define wr_lights 29
#define windowUp 30
#define windowDown 31

//RF

#include <RCSwitch.h>
#define DATA_TX433 52
#define VINRF433 50
RCSwitch tx433 = RCSwitch();

//car:
#define ignt_status 33 //31
#define ignt_interrupt 3
#define wr_lights_status 2
#define warm_up 3000
#define helloLights 15000
#define wr_lights_off_delay 1500
//#define startButton 20
int contactflag = 0;
int startflag = 0;
//int cptStop = 0;
//int cptStart = 0;
boolean locked = false;

//Interuptions:
unsigned long previousMillis = 0;
//unsigned long previousMillis_StartButton = 0;
#define interval 2000

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "dfc6102e22554d3b85c23e4f1b437f17";

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[]  = "sl2sfr";
char user[] = "";
char pass[] = "";

// Hardware Serial on Mega, Leonardo, Micro

// or Software Serial on Uno, Nano
//#include <SoftwareSerial.h>
//SoftwareSerial SerialAT(2, 3); // RX, TX

TinyGsm modem(SerialAT);
IPAddress local;
WidgetTerminal termin(V6);
WidgetMap sky_map(V0);
BlynkTimer timer;


BLYNK_WRITE(V3)
{
  int pinData = param.asInt();

  if (pinData)
  {
    starteng();
    termin.println("----------------");
    termin.println("Starting Eng....");
    termin.println("----------------");
  }
}

BLYNK_WRITE(V4)
{
  int pinData = param.asInt();

  if (pinData)
  {
    Stopengine();
    termin.println("----------------");
    termin.println("Stopping Eng....");
    termin.println("----------------");
    Blynk.virtualWrite(3, LOW);
    Blynk.virtualWrite(4, LOW);
    Blynk.virtualWrite(5, LOW);
  }
}

BLYNK_WRITE(V5)
{
  int pinData = param.asInt();

  if (pinData)
  {
    contact_fun();
    termin.println("----------------");
    termin.println("Contact on....");
    termin.println("----------------");

  }
}

BLYNK_WRITE(V6)
{
  String str = param.asStr();
  str.toLowerCase();

  switch (str[0]) {
    case 'l':
      sky_map.clear();
      indexMap = 0;
      prevLat = 0;
      prevLon = 0;
      updatelocation();
      break;
    case 'r':
      digitalWrite(relay_x, LOW);
      break;
    case 'h':
      termin.println("---------------------");
      termin.println("Yeap Still kickin....");
      termin.println("---------------------");
      break;
    case 'g':
      ghost = !ghost;
      termin.println("---------------------");
      termin.println("Ghost mode: " + ghost);
      termin.println("---------------------");
      break;
  }



}

BLYNK_WRITE(V7)
{
  int pinData = param.asInt();

  if (pinData)
  {
    locks();
    termin.println("----------------");
    termin.println("Locks command....");
    termin.println("----------------");
    Blynk.virtualWrite(7, LOW);
  }
}

BLYNK_WRITE(V8)
{
  int pinData = param.asInt();

  if (pinData)
  {
    openback();
    termin.println("----------------");
    termin.println("Back command....");
    termin.println("----------------");
    Blynk.virtualWrite(8, LOW);
  }
}

BLYNK_WRITE(V9)
{
  int pinData = param.asInt();

  if (pinData)
  {
    termin.println("----------------");
    termin.println("Open Windwos....");
    tx433.send("111110100010000011100001");
    termin.println("----------------");
  }
}

BLYNK_WRITE(V10)
{
  int pinData = param.asInt();

  if (pinData)
  {
    termin.println("----------------");
    termin.println("Close Windows...");
    tx433.send("111110100010000011100010");
    termin.println("----------------");
  }
}

BLYNK_WRITE(V12)
{
  int pinData = param.asInt();

  if (pinData)
  {
    termin.println("----------------");
    termin.println("Close Windows...");
    tx433.send("111110100010000011101000");
    termin.println("----------------");
  }
}




void updatelocation() {
  gps_fixstatus = modem.getGPS(&gps_latitude, &gps_longitude, &gps_speed, &gps_altitude, &gps_view_satellites, &gps_used_satellites);
  if ( gps_fixstatus ) {
    /*
        Serial.print("#GPS Location: LAT: ");
        Serial.println(gps_latitude, 5);
        Serial.print(" LONG: ");
        Serial.println(gps_longitude, 5);
        Serial.print(" SPEED: ");
        Serial.println(gps_speed);
        Serial.print(" ALTITUDE: ");
        Serial.println(gps_altitude);
        Serial.print(" USED SATELITES: ");
        Serial.println(gps_view_satellites);
        Serial.print(" VIEWED STELITES: ");
        Serial.println(gps_used_satellites);
        Serial.println("-----------------");
    */
    Blynk.virtualWrite(V1, gps_speed);
    distancy( prevLat, prevLon, gps_latitude, gps_longitude, gps_altitude);
    if (ghost) {
      timer.deleteTimer(trackingInterval);
      sky_map.clear();
    }

    //distance_on_geoid( 43.589319723065486, 1.2959382167709919, 43.588915634897155, 1.296034776295528, gps_altitude);

  }
}


/*
  double distance_on_geoid(double lat1, double lon1, double lat2, double lon2) {

  // Convert degrees to radians
  lat1 = lat1 * M_PI / 180.0;
  lon1 = lon1 * M_PI / 180.0;

  double lat_2 = lat2 * M_PI / 180.0;
  double lon_2 = lon2 * M_PI / 180.0;

  // radius of earth in metres
  double r = 6378100;

  // P
  double rho1 = r * cos(lat1);
  double z1 = r * sin(lat1);
  double x1 = rho1 * cos(lon1);
  double y1 = rho1 * sin(lon1);

  // Q
  double rho2 = r * cos(lat2);
  double z2 = r * sin(lat2);
  double x2 = rho2 * cos(lon2);
  double y2 = rho2 * sin(lon2);

  // Dot product
  double dot = (x1 * x2 + y1 * y2 + z1 * z2);
  double cos_theta = dot / (r * r);

  double theta = acos(cos_theta);

  return r * theta;
  }
*/
void distancy(double lat1, double lon1, double lat2, double lon2, double alti) {


  double lat_D_Rad = (lat2 - lat1) * M_PI / 180.0;
  double lon_D_Rad = (lon2 - lon1) * M_PI / 180.0;
  lat1 = lat1 * M_PI / 180.0;
  double a = pow(sin(lat_D_Rad / 2), 2)  + pow(sin(lon_D_Rad / 2), 2) * cos(lat1) * cos(lat2 * M_PI / 180.0);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = earthRadiusKm * c ;


  if (distance >= distance_between_two_wayPoint) {
    // Serial.println("Adding new point...");

    sky_map.location(indexMap, lat2, lon2, String(day()) + "/" + String(month()) + " " + String(hour() + 2) + ":" + String(minute()) + ":" + String(second()) + " | " + String(gps_speed) + " Km/h");
    prevLat = lat2;
    prevLon =  lon2;
    indexMap++;

    Blynk.virtualWrite(V2, alti);
    termin.print("Latitude: ");
    termin.println(lat2, 4);
    termin.print("Longitude: ");
    termin.println(lon2, 4);
    termin.print("Altitude: ");
    termin.println(alti);
    csq = modem.getSignalQuality();
    termin.print("Signal quality: ");
    termin.println(csq);
    termin.println("----------------");
  }

}


void aux_starter() {
  digitalWrite(starter, 1);
}

void startup_lights_off() {
  digitalWrite(signes, 1);
}

void startup_lights_on() {
  int igt_val = digitalRead(ignt_status);
  if (igt_val == LOW) {
    Serial.println("WelcomeLights");
    digitalWrite(signes, 0);
    timer.setTimeout(helloLights, startup_lights_off);
  }
}

void wr_lights_on_func() {
  digitalWrite(wr_lights, 0);
  timer.setTimeout(400, wr_lights_off_func);
}

void wr_lights_off_func() {
  digitalWrite(wr_lights, 1);
}

void contact_fun() {
  Serial.println("Commande de contact est reçue");
  contactflag = 1;
  startflag = 1;
  Blynk.virtualWrite(5, HIGH);
  wr_lights_on_func();
  digitalWrite(contact, 0);
  timer.setTimeout(wr_lights_off_delay, wr_lights_on_func);
  Serial.println("\nDémarrage contact OK");

}

void demarreur() {
  startflag = 1;
  digitalWrite(starter, 0);
  timer.setTimeout(1200, aux_starter);
}


void starteng() {
  Serial.println("Commande de démarrage est reçue");
  contactflag = digitalRead(ignt_status);
  Blynk.virtualWrite(3, HIGH);
  if (!contactflag && !startflag ) {
    contact_fun();
    timer.setTimeout(warm_up, demarreur);
  }
}

void Stopengine()
{
  Serial.println("Commande d'étendre est reçue");
  if ( contactflag || startflag && gps_speed <= 5.0) {
    digitalWrite(contact, 1);
    contactflag = 0;
    startflag = 0;
    Blynk.virtualWrite(3, LOW);
    Blynk.virtualWrite(5, LOW);
    Serial.println("étendre OK");
  }

}

void closeback() {
  digitalWrite(back, 1);
}

void openback() {
  wr_lights_on_func();
  digitalWrite(back, 0);
  timer.setTimeout(600, closeback);
  timer.setTimeout(wr_lights_off_delay, wr_lights_on_func);

  Serial.println("Coffre OK");
}

void aux_locks() {
  digitalWrite(doorslock, 1);
}

void locks() {
  wr_lights_on_func();
  digitalWrite(doorslock, 0);
  timer.setTimeout(500, aux_locks);
  timer.setTimeout(wr_lights_off_delay, wr_lights_on_func);
  locked = !locked;
  Blynk.virtualWrite(11, locked);
  Serial.println("Fermeture OK");
}

void checkup() {
  contactflag = digitalRead(ignt_status);
  sky_map.clear();
  indexMap = 0;
  prevLat = 0;
  prevLon = 0;
  if (contactflag) {
    Blynk.virtualWrite(5, HIGH);
    Blynk.virtualWrite(3, HIGH);
    trackingInterval = timer.setInterval(1000L, updatelocation);
    //  Serial.println("Interval has been set");
  } else {
    Blynk.virtualWrite(5, LOW);
    Blynk.virtualWrite(3, LOW);
    timer.deleteTimer(trackingInterval);
    //  Serial.println("Interval is removed");

    updatelocation();
  }

}

void updateIgn() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    timer.setTimeout(200, checkup);
  }

}

void setupGPS() {
  gps_fixstatus = modem.getGPS(&gps_latitude, &gps_longitude, &gps_speed, &gps_altitude, &gps_view_satellites, &gps_used_satellites);
  if ( gps_fixstatus ) {
    sky_map.clear();
    if (modem.getGPSTime(&y, &m, &d, &h, &mm, &s)) {
      //Sync time if it's diffrent.
      setTime(h, mm, s, d, m, y);
      updatelocation();
      Serial.println("Clock is set");
    }
  } else {
    timer.setTimeout(wr_lights_off_delay, setupGPS);
  }
}
/*
  void startStop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_StartButton >= interval) {
    Serial.println("StartStop Button");
    // save the last time you blinked the LED
    previousMillis_StartButton = currentMillis;
    cptStop++;
    cptStart++;
    contactflag = digitalRead(ignt_status);
    if (!contactflag && !startflag && cptStart >= 2 ) {
      cptStart = 0;
      cptStop = 0;
      timer.setTimeout(200, starteng);
    } else if (cptStop >= 3) {
      cptStop = 0;
      cptStart = 0;
      timer.setTimeout(200, Stopengine);
    }
    // Serial.println(cptStop);
  }
  }
*/

void setup()
{
  // Debug console
  Serial.begin(115200);
  TinyGsmAutoBaud(SerialAT);
  Serial.println("Restarting modem...");

  if (!modem.restart()) {
    delay(5000);
    return;
  }
  delay(5000);

  Serial.println("\nStarting ....");
  randomSeed(analogRead(5));
  pinMode(contact, OUTPUT);
  pinMode(starter, OUTPUT);
  pinMode(doorslock, OUTPUT);
  pinMode(relay_x, OUTPUT);
  pinMode(back, OUTPUT);
  pinMode(signes, OUTPUT);
  pinMode(wr_lights, OUTPUT);
  pinMode(ignt_status, INPUT);
  // pinMode(startButton, INPUT_PULLUP);
  pinMode(VINRF433, OUTPUT);
  pinMode(gateController, OUTPUT);
  pinMode(windowUp, OUTPUT);
  pinMode(windowDown, OUTPUT);

  digitalWrite(VINRF433, HIGH);
  digitalWrite(contact, 1);
  digitalWrite(windowDown, 0);
  digitalWrite(windowUp, 0);
  digitalWrite(starter, 1);
  digitalWrite(doorslock, 1);
  digitalWrite(relay_x, 1);
  digitalWrite(back, 1);
  digitalWrite(signes, 1);
  digitalWrite(wr_lights, 1);
  digitalWrite(gateController, 1);
  Serial.println("Démarrage des relais ");
  attachInterrupt(digitalPinToInterrupt(wr_lights_status), startup_lights_on, HIGH);
  attachInterrupt(digitalPinToInterrupt(ignt_interrupt), updateIgn, CHANGE);
  tx433.enableTransmit(DATA_TX433);
  //attachInterrupt(digitalPinToInterrupt(startButton), startStop, HIGH);


  // Set GSM module baud rate





  // Restart takes quite some time
  // To skip it, call init() instead of restart()

  DBG("Modem:", modem.getModemInfo());
  /*
    DBG("Waiting for network...");
    if (!modem.waitForNetwork()) {
      delay(10000);
      return;
    }

    if (modem.isNetworkConnected()) {
      DBG("Network connected");
    }

    bool res = modem.isGprsConnected();
    DBG("GPRS status:", res ? "connected" : "not connected");

    String cop = modem.getOperator();
    DBG("Operator:", cop);


    // This is only supported on SIMxxx series
    String gsmLoc = modem.getGsmLocation();
    DBG("GSM location:", gsmLoc);

  */

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  modem.enableGPS();
  Blynk.begin(auth, modem, apn, user, pass);
  local = modem.localIP();
  csq = modem.getSignalQuality();
  Blynk.virtualWrite(12, locked);
  sky_map.clear();
  termin.clear();
  termin.print("IP: ");
  termin.println(local);
  termin.print("Signal quality: ");
  termin.println(csq);
  termin.println("----------------");
  setupGPS();
  //timer.setInterval(1000L, updatelocation);
  // timer.setInterval(5000L, checkup);


}

void loop()
{

  if (Blynk.connected()) {
    Blynk.run();

  } else {
    Serial.println("GPRS Connected but Blynk isn't");
    if (modem.isGprsConnected()) {
      Serial.println("GPRS Connected");
      Blynk.begin(auth, modem, apn, user, pass);
    } else {
      digitalWrite(relay_x, LOW);
    }
  }
  timer.run();
  // You can inject your own code or combine it with other sketches.
  // Check other examples on how to communicate with Blynk. Remember
  // to avoid delay() function!

}


