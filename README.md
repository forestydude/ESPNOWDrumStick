# ESPNOWDrumStick
ESP32 Arduino Code for MPU6050 using ESP NOW to communicate with server that acts a MIDI device (TinyUSB)

This code was originally designed for the use with the MPU6050 module that senses acceleration. When a threshold is met (work in progress), the client device sends a message to the server with a hardcoded note and velocity that is mapped to the acceleration.

The client is currently hardcoded for the drumstick MIDI channel.

The MAC address must be changed on the client device for the server MAC address of the user's ESP32 running the server code. There is code available online that can print ESP32 MAC address to serial. (Built-in MAC address reporting and setup is currently a work in progress.)
