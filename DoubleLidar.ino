#include <Wire.h>
#include <VL53L0X.h>
#include <Ethernet.h>

#define PI 3.1415926535897932384626433832795
#define OFFSET_RIGHT -40.0 
#define OFFSET_LEFT   0.0
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

void setup() {
  Serial.begin(9600);
  Wire.begin();


  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }  

  // Initialize the Ethernet server
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());

  initializeLidarSystem();
}

void loop() {
  int distance_left;
  int distance_right;
  double turn_angle;
  bool leftReadSuccess;
  bool rightReadSuccess;

  calculateDistanceAndAngle(distance_left, distance_right, turn_angle, leftReadSuccess, rightReadSuccess);
  EthernetClient client = server.available();
  if (client) {
    readClientData(client);
    writeToClient(client, distance_left, distance_right, turn_angle);
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }

  serialPrintvalues(distance_left, distance_right, turn_angle, leftReadSuccess, rightReadSuccess);
    
  delay(250);
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
}

void readClientData(EthernetClient client) {
  int len = client.available();
  if (len > 0) {
    byte buffer[80];
    if (len > 80) len = 80;
    client.read(buffer, len);
    Serial.write(buffer, len); // show in the serial monitor (slows some boards)
  }
}

bool initializeLidarSystem() {
  bool returnVal;
  
  // disable the left lidar by setting pin 5 to low
  pinMode(5, OUTPUT);

  // Initialize the right lidar and reset its address to 0x30
  //returnVal = lox_right.begin(0x30);
  lox_right.setAddress(0x42);
  returnVal = lox_right.init();

  // enable the left lidar by setting pinmode on pin 5 to INPUT and initialize
  pinMode(5, INPUT);
  delay(10);
  returnVal = returnVal && lox_left.init();

  lox_left.setTimeout(500);
  lox_right.setTimeout(500);

  return returnVal;
}

double calculateAngle(double right, double left) {
  double b = LIDAR_SEPERATION; //distance between lidars in mm
  double c = sqrt((pow(b,2)) + (pow(right,2)));
  double d = sqrt((pow(c,2)) + (pow(left,2)));
  double B = atan(b/right);
  double A = ((PI/2) - B);
  double D = ((PI/2) - A);
  double E = asin((left*sin(D))/(d));
  double F = (PI - E - B);
  double X = ((PI/2) - F);
  return (X*(180/PI));
}

bool calculateDistanceAndAngle(int &left, int &right, double &turn_angle, bool &leftReadSuccess, bool &rightReadSuccess) {
  turn_angle = 0;
  // Read the right lidar and store the success
  right = lox_right.readRangeSingleMillimeters() + OFFSET_RIGHT;
  rightReadSuccess = true; //measure_right.RangeStatus != 4;

  // Read the left lidar and store the success  
  left  = lox_left.readRangeSingleMillimeters();
  leftReadSuccess = true; //measure_left.RangeStatus != 4;

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
