//Server set USB Mode to USB-OTG
#include <esp_now.h>
#include <WiFi.h>
#include "USB.h"
#include "esp32-hal-tinyusb.h"
#include "tusb.h"
#include <Control_Surface.h>

USBMIDI_Interface midi;

void sendNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
  uint8_t cable = 0;  // Usually 0 unless you're multiplexing
  uint8_t cin = 0x9;  // Code Index Number for Note On

  uint8_t packet[4] = {
    (cable << 4) | cin,
    0x90 | ((channel - 1) & 0x0F),  // Status byte: Note On + channel
    note,
    velocity
  };

  tud_midi_stream_write(cable, packet, 4);
}

void sendNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
  uint8_t cable = 0;
  uint8_t cin = 0x8;  // Code Index Number for Note Off

  uint8_t packet[4] = {
    (cable << 4) | cin,
    0x80 | ((channel - 1) & 0x0F),  // Status byte: Note Off + channel
    note,
    velocity
  };

  tud_midi_stream_write(cable, packet, 4);
}


//USBMIDI MIDI;  // Direct USB MIDI object

typedef struct midi_message {
  int id;
  int note;
  int velocity;
} midi_message;

midi_message MIDIData;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  const uint8_t *mac_addr = recv_info->src_addr;  // Extract MAC address if needed

  memcpy(&MIDIData, incomingData, sizeof(MIDIData));

  Serial.printf("Board %d: x=%d, y=%d\n", MIDIData.id, MIDIData.note, MIDIData.velocity);

  sendNoteOn(MIDIData.id, MIDIData.note, MIDIData.velocity);
  delay(100);
  sendNoteOff(MIDIData.id, MIDIData.note, MIDIData.velocity);
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  
  USB.begin();  // Initialize USB stack
  
  //MIDI.begin(); // Initialize USB MIDI
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  delay(10);  // Passive loop
}
