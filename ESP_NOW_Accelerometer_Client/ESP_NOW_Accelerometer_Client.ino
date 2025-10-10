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
    int velocity;
} midi_message;

midi_message MIDIData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);  // halt if sensor not found
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
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

  if (abs(a.acceleration.x) > 19.8 || abs(a.acceleration.y) > 9.8) {
    // Set values to send
    MIDIData.id = 10;
    MIDIData.note = 56;
    MIDIData.velocity = abs(map(a.acceleration.x,0,64,1,127));

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &MIDIData, sizeof(MIDIData));
    
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    };
    delay(100); // debounce
  }
  delay(10); // sampling rate
}