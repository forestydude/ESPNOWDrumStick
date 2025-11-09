//Client
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// REPLACE WITH THE RECEIVER'S MAC Address 
uint8_t broadcastAddress[] = {0xcc,0x8d,0xa2,0xed,0xce,0xb0};

// Structure to send data
// Must match the receiver structure
typedef struct midi_message {
    int id; // must be unique for each sender board
    int note;
    float velocity;
} midi_message;

midi_message MIDIData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent uncomment to debug
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//Current gravity in x, y, z components
float gravityX = 0;
float gravityY = 0;
float gravityZ = 0;
//Scalar component of acceleration in direction of gravity is stored in a buffer array 
const int bufferSize = 5; //Adjust buffer size to change how many samples of data are used in acceleration min and averages
float componentBuffer[bufferSize] = {0};
float yAccelerationBuffer[bufferSize] = {0}; //y acceleration (the acceleration that the drumstick points) buffer used for integrating velocity
int bufferIndex = 0;
int lastbufferIndex = 0;
//Default threshold that drumstick swing must beat to trigger a message (positive)
float gravityComponentThresholdDefault = 100;
float gravityComponentThreshold = gravityComponentThresholdDefault;
//Threshold that square of acceleration must be within +/- to reset the gravity vector (upon rotation) 
float gravityResetThresholdSquare = 10;
//Do not allow a message for a strike to be sent on boot
bool allowStrike = 0;
float earthGravitySquare = 9.8*9.8;
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);  // halt if sensor not found
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelerationMagnitudeSquare = pow(a.acceleration.x,2) + pow(a.acceleration.y,2) + pow(a.acceleration.z,2);
  float gravityComponent = a.acceleration.x*gravityX + a.acceleration.y*gravityY + a.acceleration.z*gravityZ;

  if (accelerationMagnitudeSquare > earthGravitySquare - gravityResetThresholdSquare && accelerationMagnitudeSquare < earthGravitySquare + gravityResetThresholdSquare) {
    gravityX = a.acceleration.x;
    gravityY = a.acceleration.y;
    gravityZ = a.acceleration.z;
  }
  // Store gravity component and y acceleration in buffers
  componentBuffer[bufferIndex] = gravityComponent;
  yAccelerationBuffer[bufferIndex] = a.acceleration.y;
  lastbufferIndex = (bufferIndex + bufferSize - 1) % bufferSize;
  bufferIndex = (bufferIndex + 1) % bufferSize;


  // Compute minimum and maximum gravity component in buffer, average y acceleration, and net y velocity
  float gravityComponentMin = componentBuffer[0];
  float gravityComponentMax = componentBuffer[0];
  float netVelocityY = 0;

  for (int i = 0; i < bufferSize; i++) {
    if (componentBuffer[i] < gravityComponentMin) gravityComponentMin = componentBuffer[i];
    if (componentBuffer[i] > gravityComponentMax) gravityComponentMax = componentBuffer[i];
    netVelocityY += yAccelerationBuffer[i] * 0.01; // 10ms sampling
  }
  // if velocity is positive, acceleration is due to pull-back of stick or recoil on stick
  if (netVelocityY > 0) {
    // if pull-back is stronger than default, set new maximum that downward swing must beat
    // this prevents the recoil on an upward swing from triggering a hit (recoil should be less than initial input, while pull-back for a downward swing is not as strong as swing)
    if (gravityComponentMax > gravityComponentThresholdDefault) {
      gravityComponentThreshold = gravityComponentMax;
    }
  }
  // uncomment for debugging

  // Serial.print("x:");
  // Serial.print(a.acceleration.x);
  // Serial.print(" y:");
  // Serial.print(a.acceleration.y);
  // Serial.print(" z:");
  // Serial.print(a.acceleration.z);
  // Serial.print(" vnet:");
  // Serial.print(netVelocityY);
  // Serial.print(" thres:");
  // Serial.print(gravityComponentThreshold);
  // Serial.print(" gda:");
  // Serial.println(gravityComponent);

  // Restore strike on positive acceleration
  if (gravityComponent > 0) {
    allowStrike = 1;
  }
  // Downward swing detection
  // Swing must be below negative gravity threshold, componentBuffer[lastbufferIndex] == gravityComponentMin may not be necessary for small sample size
  // Once gravityComponent decreases, trigger swing only if y velocity is down (stick moves in direction that points down)
  // Only send message if allow strike is true
  if (gravityComponentMin < -gravityComponentThreshold && componentBuffer[lastbufferIndex] == gravityComponentMin && gravityComponent > componentBuffer[lastbufferIndex] && netVelocityY < -0.1 && allowStrike) {
    MIDIData.id = 10;
    MIDIData.note = 56;
    MIDIData.velocity = abs(map(gravityComponentMin, 0, 64, 1, 127));
    esp_now_send(broadcastAddress, (uint8_t *) &MIDIData, sizeof(MIDIData));
    //Prevents multiple strikes. A long sweeping strike can cause multiple hits due to deceleration
    allowStrike = 0;
  }
  delay(10);
}
