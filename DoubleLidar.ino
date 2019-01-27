#include "Adafruit_VL53L0X.h"
#define PI 3.1415926535897932384626433832795

Adafruit_VL53L0X lox_right = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_left = Adafruit_VL53L0X();

double offset_1 = -40 ;
double offset_2 = -0  ;

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  delay(10);
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox_right.begin(0x30)) {
    Serial.println(F("Failed to boot right LIDAR"));
    while(1);
  }

   digitalWrite(5, HIGH);
   
    Serial.println("Adafruit VL53L0X test");
  if (!lox_left.begin()) {
    Serial.println(F("Failed to boot left LIDAR"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure_right;
  VL53L0X_RangingMeasurementData_t measure_left;
    
  Serial.print("Reading a measurement... ");
  lox_right.rangingTest(&measure_right, false); // pass in 'true' to get debug data printout!
  lox_left.rangingTest(&measure_left, false); // pass in 'true' to get debug data printout!
  int distance_right = measure_right.RangeMilliMeter + offset_1;
  int distance_left = measure_left.RangeMilliMeter + offset_2;
  double b = 265; //distance between lidars
  double a = measure_right.RangeMilliMeter + offset_1;
  double e = measure_left.RangeMilliMeter + offset_2;
  double c = sqrt((pow(b,2)) + (pow(a,2)));
  double d = sqrt((pow(c,2)) + (pow(e,2)));
  double B = atan(b/a);
  double A = ((PI/2) - B);
  double D = ((PI/2) - A);
  double E = asin((e*sin(D))/(d));
  double F = (PI - E - B);
  double X = ((PI/2) - F);
  double turn_angle = (X*(180/PI));

  Serial.println("turn_angle");
  Serial.println(turn_angle);
  

  if (measure_right.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Right Distance (mm): "); Serial.println(distance_right);
  } else {
    Serial.println(" right out of range ");
  }

   if (measure_left.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Left Distance (mm): "); Serial.println(distance_left);
  } else {
    Serial.println(" left out of range ");
  }
    
  delay(250);
}
