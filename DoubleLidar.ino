#include <Wire.h>
#include <VL53L0X.h>
#include <Ethernet.h>

#define PI 3.1415926535897932384626433832795
#define OFFSET_RIGHT -15.0
#define OFFSET_LEFT -10.0
#define LIDAR_SEPERATION 265.0

VL53L0X lox_right;
VL53L0X lox_left;


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0xDE, 0xAD, 0xBD, 0xEF, 0xFE, 0xAD};
IPAddress ip(10, 48, 59, 17);

// Initialize the Ethernet server library with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

const String outOfRange = "Out of Range!";

void(* resetFunc) (void) = 0;//declare reset function at address 0

void setup() {
  Wire.begin();        // Initialize the I2C library
  Serial.begin(9600);  // Initialize the serial output for the arduino
  
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1);  }  

  // Initialize the lidar system
  Serial.println("calling initializeLidarSystem ");
  bool lidarInitSuccess = initializeLidarSystem();
  Serial.println("return from initializeLidarSystem ");
  
  // Initialize the Ethernet server
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  int distance_left;
  int distance_right;
  double turn_angle;
  bool leftReadSuccess;
  bool rightReadSuccess;

    
  // See if we have a client knocking at the door
  EthernetClient client = server.available();
  if (client) {
    // Read the lidars and calculate the angle necessary to turn the robot to approach the target
    calculateDistanceAndAngle(distance_left, distance_right, turn_angle, leftReadSuccess, rightReadSuccess);
    serialPrintvalues(distance_left, distance_right, turn_angle, leftReadSuccess, rightReadSuccess);

    //Read in the data being passed to us.  (Might be useful later on)
    readClientData(client);

    // Write the data from the lidars back to the client.
    writeToClient(client, distance_left, distance_right, turn_angle);

    // close the connection:
    client.stop();
  }
}


void writeToClient(EthernetClient client, int left, int right, double angle) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");  // the connection will be closed after completion of the response
  client.println();
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.print("[[[");
  client.print(left); client.print(',');
  client.print(right); client.print(',');
  client.print(angle);
  client.print("]]]");
  client.println("</html>");    
  // give the web browser time to receive the data
  delay(5);
}
void writeResetToClient(EthernetClient client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");  // the connection will be closed after completion of the response
  client.println();
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.print("Reset");
  client.println("</html>");    
  // give the web browser time to receive the data
  delay(5);
}

void readClientData(EthernetClient client) {
  int len = client.available();
  if (len > 0) {
    byte buffer[80];
    if (len > 80) len = 80;
    client.read(buffer, len);
    String strbuffer = buffer;
    if(strbuffer.indexOf("reset") != -1) {    
      // Write the data from the lidars back to the client.
      writeResetToClient(client);

    // close the connection:
      client.stop();

      resetFunc();  //call reset
      //Serial.println("hello world");
    }
    Serial.print(strbuffer); // show in the serial monitor (slows some boards)
  }
}


bool initializeLidarSystem() {
  bool returnVal;
  
  // disable the left lidar by setting pin 5 to low
  pinMode(8, OUTPUT);

  // Initialize the right lidar and reset its address to 0x30
  //returnVal = lox_right.begin(0x30);
  lox_right.setAddress(0x42);
  returnVal = lox_right.init();
 
  // enable the left lidar by setting pinmode on pin 5 to INPUT and initialize
  pinMode(8, INPUT);
  delay(10);
  returnVal = returnVal && lox_left.init();

  lox_left.setTimeout(500);
  lox_right.setTimeout(500);

  //lox_left.setSignalRateLimit(0.1);
  //lox_right.setSignalRateLimit(0.1);
  //lox_left.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  //lox_left.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  //lox_right.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  //lox_right.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  //lox_left.setMeasurementTimingBudget(200000);
  //lox_right.setMeasurementTimingBudget(200000);
}

double calculateAngle(double right, double left) {
  double b = LIDAR_SEPERATION; //distance between lidars in mm
  double a = right;
  double e = left;
  double dif = (a - e);
  double X = atan(dif/LIDAR_SEPERATION);
  double turn_angle = X*(180/PI);
  return (turn_angle);
}

bool calculateDistanceAndAngle(int &left, int &right, double &turn_angle, bool &leftReadSuccess, bool &rightReadSuccess) {
   turn_angle = 0;
  // Read the right lidar and store the success
  right = lox_right.readRangeSingleMillimeters() + OFFSET_RIGHT;
  rightReadSuccess = ((right > 0) && (right < 8100));

  // Read the left lidar and store the success  
  left  = lox_left.readRangeSingleMillimeters() + OFFSET_LEFT;
  leftReadSuccess = ((left > 0) && (left < 8100));

  // If both reads of the lidars were successful, calculate the angle to the target
  if (rightReadSuccess && leftReadSuccess) {
    turn_angle = calculateAngle(right, left);
  }

  return (leftReadSuccess && rightReadSuccess);
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
