#include <WiFi.h>
#include <WebServer.h>

#define SENSOR_LEFT_PIN 5
#define SENSOR_CENTER_PIN 4
#define SENSOR_RIGHT_PIN 2
#define SENSOR_FAR_LEFT_PIN 18
#define SENSOR_FAR_RIGHT_PIN 15

#define BIN1 26
#define BIN2 25
#define PWMB 33
#define AIN1 14
#define AIN2 12
#define PWMA 13
#define STBY 27

#define PWMA_CHANNEL 0
#define PWMB_CHANNEL 1

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

const int ledPin = 23;

int far_left;
int left;
int center;
int right;
int far_right;

int Kp = 0;
int Ki = 0;
int Kd = 0;
int previousError = 0;
int integral = 0;
int error = 0;
int stop = 0;
int baseSpeed = 0;// Base speed of the motors (0-255)

const char* ssid = "THS";
const char* password = "asdf12345";

WebServer server(80);

int P_value = 0;
int I_value = 0;
int D_value = 0;
int speed_value = 0;

void handleRoot() {
  String html = "<html><body><h1>ESP32 Web Server</h1></body></html>";
  server.send(200, "text/html", html);
}

void handleSetPID() {
  if (server.hasArg("P") && server.hasArg("I") && server.hasArg("D") && server.hasArg("Speed")) {
    P_value = server.arg("P").toInt();
    I_value = server.arg("I").toInt();
    D_value = server.arg("D").toInt();
    speed_value = server.arg("Speed").toInt();
    String response = "Received P: " + String(P_value) + ", I: " + String(I_value) + ", D: " + String(D_value) + ", Speed: " + String(speed_value);
    server.send(200, "text/plain", response);
    Kp = P_value;
    Ki = I_value;
    Kd = D_value;
    baseSpeed = speed_value;

  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void setupMotorDriver() {

  pinMode(ledPin, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);  
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, HIGH); // Enable the motor driver

  // Configure PWM channels
  //ledcSetup(PWMA_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMA, 1000, 8);

  //ledcSetup(PWMB_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, 1000, 8);
}

void setupSensor() {
  pinMode(SENSOR_LEFT_PIN, INPUT);
  pinMode(SENSOR_CENTER_PIN, INPUT);
  pinMode(SENSOR_RIGHT_PIN, INPUT);
  pinMode(SENSOR_FAR_RIGHT_PIN, INPUT);
  pinMode(SENSOR_FAR_LEFT_PIN, INPUT);
}

void MyWiFiSetup(){
  digitalWrite(ledPin, HIGH);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  //digitalWrite(ledPin, HIGH);
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.on("/", handleRoot);
  server.on("/setPID", handleSetPID);
  server.begin();
  Serial.println("Server started");
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (rightSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  ledcWrite(PWMA, rightSpeed);

  if (leftSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  ledcWrite(PWMB, leftSpeed);
}

void SensorRead(){
  left = digitalRead(SENSOR_LEFT_PIN);
  center = digitalRead(SENSOR_CENTER_PIN);
  right = digitalRead(SENSOR_RIGHT_PIN);
  far_left = digitalRead(SENSOR_FAR_LEFT_PIN);
  far_right = digitalRead(SENSOR_FAR_RIGHT_PIN);
}

void setup() {
  Serial.begin(115200);
  digitalWrite(ledPin, LOW);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.on("/", handleRoot);
  server.on("/setPID", handleSetPID);
  server.begin();
  Serial.println("Server started");

  setupMotorDriver();
  setupSensor();
}

void loop() {
  server.handleClient();
  SensorRead();
  digitalWrite(ledPin, HIGH);
  if(left == 1 && center == 0 && right == 1)
  {
    error = 0; //center on the line
  }
  else if(far_left == 0 && left == 0)
  {
    error = 10; //right on the line
  }
  else if(far_right == 0 && right == 0)
  {
    error = -10; //right on the line
  }
  else if(left == 0 && center == 0 && right == 1)
  {
    error = 1; //left and center on the line
  }
  else if(left == 0 && center == 1 && right == 1)
  {
    error = 2; //left on the line
  }
  else if(left == 1 && center == 0 && right == 0)
  {
    error = -1; //right and center on the line
  }
  else if(left == 1 && center == 1 && right == 0)
  {
    error = -2; //right on the line
  }
  else if(left == 1 && center == 1 && right == 1)
  {
    error = previousError; //mid sensors not on the line
  }


  // Serial.print("Error: ");
  // Serial.println(error);
  // Serial.print("  FAR LEFT:"); 
  // Serial.print(digitalRead(SENSOR_FAR_LEFT_PIN)); 
  // Serial.print(" LEFT:"); 
  // Serial.print(digitalRead(SENSOR_LEFT_PIN)); 
  // Serial.print(" CENTER:"); 
  // Serial.print(digitalRead(SENSOR_CENTER_PIN)); 
  // Serial.print(" RIGHT:"); 
  // Serial.print(digitalRead(SENSOR_RIGHT_PIN));
  // Serial.print(" FAR RIGHT:"); 
  // Serial.println(digitalRead(SENSOR_FAR_RIGHT_PIN)); 

  Serial.print("P : ");
  Serial.print(P_value);
  Serial.print("  I : ");
  Serial.print(I_value);
  Serial.print("  D : ");
  Serial.print(D_value);
  Serial.print("  Speed : ");
  Serial.println(speed_value);

  integral += error;
  float derivative = error - previousError;
  float output = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = baseSpeed - output;
  int rightSpeed = baseSpeed + output;

  // Constrain the speeds to be within the valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // if(stop !=  1)
  setMotorSpeed(leftSpeed, rightSpeed);

  previousError = error;

  // Serial.print("PID Output: ");
  // Serial.print(output);
  // Serial.print(" left speed: ");
  // Serial.print(leftSpeed);
  // Serial.print(" right speed: ");
  // Serial.println(rightSpeed);
  //delay(10); // Adjust delay as needed
}