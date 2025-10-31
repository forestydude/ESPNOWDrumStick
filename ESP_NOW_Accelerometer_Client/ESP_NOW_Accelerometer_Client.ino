//Client
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// REPLACE WITH THE RECEIVER'S MAC Address 
uint8_t broadcastAddress[] = {0xcc,0x8d,0xa2,0xed,0xce,0xb0};

// Structure example to send data
// Must match the receiver structure
typedef struct midi_message {
    int id; // must be unique for each sender board
    int note;
    float velocity;
} midi_message;

midi_message MIDIData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
float gx = 0;
float gy = 0;
float gz = 0;
float gv = 0;
float gda = 0;
float gm = 0;
float threshold = 10;
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
  gv = gx*gx + gy*gy + gz*gz;
  Serial.print("x:");
  Serial.print(a.acceleration.x);
  Serial.print(" y:");
  Serial.print(a.acceleration.y);
  Serial.print(" z:");
  Serial.print(a.acceleration.z);
  gm = pow(a.acceleration.x,2) + pow(a.acceleration.y,2) + pow(a.acceleration.z,2);
  gda = a.acceleration.x*gx + a.acceleration.y*gy + a.acceleration.z*gz;
  if (gm > 9.8*9.8-threshold && gm < 9.8*9.8+threshold) {
    gx = a.acceleration.x;
    gy = a.acceleration.y;
    gz = a.acceleration.z;
  }
  Serial.print(" gda:");
  Serial.println(gda);
  if (gda/gv > 2) {
    // Set values to send
    MIDIData.id = 10;
    MIDIData.note = 56;
    MIDIData.velocity = abs(map(gda/gv,0,64,1,127));

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &MIDIData, sizeof(MIDIData));
    
    if (result == ESP_OK) {
      //Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    };
    delay(100); // debounce
  }
  delay(10); // sampling rate
}
