#pragma once

// WiFi credentials — edit before flashing
#define WIFI_SSID "YourSSID"
#define WIFI_PASS "YourPassword"

// Port the HTTP dashboard listens on
#define WIFI_HTTP_PORT  80

// Port stats JSON is broadcast to on the local subnet every 500 ms
// Listen with: nc -ulk 4210  or any UDP client
#define WIFI_UDP_PORT   4210
