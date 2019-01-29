#include "VL53L0X.h"
#define PI 3.1415926535897932384626433832795

VL53L0X lox_right = VL53L0X();
VL53L0X lox_left = VL53L0X();

#define OFFSET_RIGHT -40 
#define OFFSET_LEFT   0
#define LIDAR_SEPERATION 265

const String outOfRange = "Out of Range!";

void setup() {
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }  

  initializeLidarSystem();
}


void loop() {
  double turn_angle = 0;

  // Read the right lidar and store the success
  int distance_right = lox_right.readRangeSingleMillimeters() + OFFSET_RIGHT;
  bool rightReadSuccess = !lox_right.timeoutOccurred();

  // Read the left lidar and store the success  
  int distance_left  = lox_left.readRangeSingleMillimeters() + OFFSET_LEFT;
  bool leftReadSuccess = !lox_left.timeoutOccurred();

  // If both reads of the lidars were successful, calculate the angle to the target
  if (rightReadSuccess && leftReadSuccess) {
    turn_angle = calculateAngle(distance_right, distance_left);
  }

  serialPrintvalues(int distance_left, int distance_right, double turn_angle, bool leftReadSuccess, bool rightReadSuccess);
    
  delay(250);
}

bool initializeLidarSystem() {
  bool returnVal;
  
  // disable the left lidar by setting pin 5 to low
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  delay(10);
  
  // Initialize the right lidar and reset its address to 0x30
  returnVal = lox_right.init();
  lox_right.setAddress(0x30);

  // enable the left lidar by setting pin 5 to high and initialize
  digitalWrite(5, HIGH);
  returnVal = returnVal && lox_left.init();

  // lower the return signal rate limit (default is 0.25 MCPS)
  lox_left.setSignalRateLimit(0.1);
  lox_right.setSignalRateLimit(0.1);
  
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  lox_left.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox_left.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  lox_right.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox_right.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
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
    { Serial.println(outOfRange); }

  // Print the right lidar value
  Serial.print(F("right lidar mm:  "));
  if (rightSuccess) 
    { Serial.print(right); }
  else 
    { Serial.println(outOfRange); }
  
  // Print the angle if both lidar reads were successful
  if (leftSuccess && rightSuccess) {
    Serial.print(F("turn_angle:      "));
    Serial.println(angle);
  }
}
