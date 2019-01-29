#include "Adafruit_VL53L0X.h"
//#include <Ethernet.h>

#define PI 3.1415926535897932384626433832795

Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left = Adafruit_VL53L0X();

#define OFFSET_RIGHT -40.0 
#define OFFSET_LEFT   0.0
#define LIDAR_SEPERATION 265.0

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//byte mac[] = { 0xDE, 0xAD, 0xBD, 0xEF, 0xFE, 0xAD};
//IPAddress ip(10, 48, 59, 17);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
//EthernetServer server(80);

const String outOfRange = "Out of Range!";

void setup() {
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }  
  //Ethernet.begin(mac, ip);
  //server.begin();
  //Serial.print("server is at ");
  //Serial.println(Ethernet.localIP());

  initializeLidarSystem();
}


void loop() {
  double turn_angle = 0;

  VL53L0X_RangingMeasurementData_t measure_right;
  VL53L0X_RangingMeasurementData_t measure_left;
  lox_right.rangingTest(&measure_right, false); // pass in 'true' to get debug data printout!
  lox_left.rangingTest(&measure_left, false); // pass in 'true' to get debug data printout!

  
  // Read the right lidar and store the success
  int distance_right = measure_right.RangeMilliMeter + OFFSET_RIGHT;
  bool rightReadSuccess = measure_right.RangeStatus != 4;

  // Read the left lidar and store the success  
  int distance_left  = measure_left.RangeMilliMeter + OFFSET_LEFT;
  bool leftReadSuccess = measure_left.RangeStatus != 4;

  // If both reads of the lidars were successful, calculate the angle to the target
  if (rightReadSuccess && leftReadSuccess) {
    turn_angle = calculateAngle(distance_right, distance_left);
  }

  serialPrintvalues(distance_left, distance_right, turn_angle, leftReadSuccess, rightReadSuccess);
    
  delay(250);
}

bool initializeLidarSystem() {
  bool returnVal;
  
  // disable the left lidar by setting pin 5 to low
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  delay(10);

  // Initialize the right lidar and reset its address to 0x30
  returnVal = lox_right.begin(0x30);
 
  // enable the left lidar by setting pin 5 to high and initialize
  digitalWrite(5, HIGH);
  returnVal = returnVal && lox_left.begin();
}

double calculateAngle(double a, double e) {
  double b = LIDAR_SEPERATION; //distance between lidars in mm
  double c = sqrt((pow(b,2)) + (pow(a,2)));
  double d = sqrt((pow(c,2)) + (pow(e,2)));
  double B = atan(b/a);
  double A = ((PI/2) - B);
  double D = ((PI/2) - A);
  double E = asin((e*sin(D))/(d));
  double F = (PI - E - B);
  double X = ((PI/2) - F);
  return (X*(180/PI));
}

void serialPrintvalues(int left, int right, double angle, bool leftSuccess, bool rightSuccess) {
  // Print the left lidar value
  Serial.print(F("left lidar mm:   ")); 
  if (leftSuccess) 
    { Serial.print(left); }
  else 
    { Serial.print(outOfRange); }

  // Print the right lidar value
  Serial.print(F(" right lidar mm:  "));
  if (rightSuccess) 
    { Serial.print(right); }
  else 
    { Serial.print(outOfRange); }
  
  // Print the angle if both lidar reads were successful
  Serial.print(F(" turn_angle:      "));
  if (leftSuccess && rightSuccess) {
    Serial.println(angle);
  }
  else{ Serial.println(outOfRange); }
}
