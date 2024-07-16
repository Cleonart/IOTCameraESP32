#include "esp_camera.h"
#include "SPI.h"
#include "driver/rtc_io.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <base64.h>

bool isStreaming = true;

/*
 * Network Instance
 * ----------------
 * SSID      : SET BY USER
 * Password  : SET BY USER
 * MAC       : DEVICE ID (UNIQUE FOR EVERY DEVICCE)
 */
const char* serverName = "https://api-central.ipxware.com/api/iot/camera/upload_2";
const char* ssid = "Aquos";
const char* password = "@ngonisomkg";
const char* root_ca = "-----BEGIN CERTIFICATE-----\nMIIFSTCCBDGgAwIBAgISA2KchTPm8S/zLzdYj2Z1sE+5MA0GCSqGSIb3DQEBCwUA\nMDMxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQwwCgYDVQQD\nEwNSMTEwHhcNMjQwNjI5MTEwMDU4WhcNMjQwOTI3MTEwMDU3WjArMSkwJwYDVQQD\nEyBhcGktYmxlc3NlZGNhZmVyZXN0by5pcHh3YXJlLmNvbTCCASIwDQYJKoZIhvcN\nAQEBBQADggEPADCCAQoCggEBAL0MGvB6/tc7uHxq6JzAbm+6mHQEpyNp8dcs1RbQ\n7Ki25hA9rI0rh0PTipJOZHQ05+JBsdKKCQLp8kxiLM+c/MJ1NDX36RFF54d8lvR6\nQx8p4ThljUkVOsCimJor5O+Mwt0MWVfMqQpmyoCBmuWRcitQRx/A2AxGdIqRoe0+\nVF44Bq6BFjBR0tb+STQAUZghs4i/XKNdIS+RU5nxybsh03lnQB2K8j+CtIABAf3w\nfFDUqKL5/X3DXsavQa77J1BuYAJfDGFZ+zqw1inUihlV6kW8reZ1ue5wpGSB2HTK\npZA3aH3W38ElF3iln36sLB9GOukq+MnSHZNFcOxal7LpPe8CAwEAAaOCAl0wggJZ\nMA4GA1UdDwEB/wQEAwIFoDAdBgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIw\nDAYDVR0TAQH/BAIwADAdBgNVHQ4EFgQUlcSGwKLPkPSpnWNr39bqIwcf0EkwHwYD\nVR0jBBgwFoAUxc9GpOr0w8B6bJXELbBeki8m47kwVwYIKwYBBQUHAQEESzBJMCIG\nCCsGAQUFBzABhhZodHRwOi8vcjExLm8ubGVuY3Iub3JnMCMGCCsGAQUFBzAChhdo\ndHRwOi8vcjExLmkubGVuY3Iub3JnLzBkBgNVHREEXTBbgiBhcGktYmxlc3NlZGNh\nZmVyZXN0by5pcHh3YXJlLmNvbYIXYXBpLWNlbnRyYWwuaXB4d2FyZS5jb22CHmFw\naS1yaXZhbGR5dGVoYW1lbi5pcHh3YXJlLmNvbTATBgNVHSAEDDAKMAgGBmeBDAEC\nATCCAQQGCisGAQQB1nkCBAIEgfUEgfIA8AB3AD8XS0/XIkdYlB1lHIS+DRLtkDd/\nH4Vq68G/KIXs+GRuAAABkGPeuLgAAAQDAEgwRgIhAN09rh5OlJP9io1Zq4Cno51/\nCx4l0iN1Cj5k6FfRdT0sAiEAw5dMBDEDA18P5Zpb67KPabaRYYj5DaH4NpXFPme1\nw+cAdQDuzdBk1dsazsVct520zROiModGfLzs3sNRSFlGcR+1mwAAAZBj3rjHAAAE\nAwBGMEQCIAGAthmgLQGTQ5mqWAIM+FoL0o7n9RlT7krF5tyGWzsGAiBFwFKOAbcC\n9SntuIk9SgWfggZFVt20xnjeBscJ1bXK9TANBgkqhkiG9w0BAQsFAAOCAQEAfu/g\nQcOaLPfvhNgBS3YSQJXaQejMbQqBqiaykO8afl3C3gorTpRvtQOnzFeqGWp1PPpk\nGxYHM81+tN/3kja0l277ZmVej4DOn60VXOdDMrJ8GfP2P120xnVmZkQJroaNXmbY\nzuQTi/vJ0OPPRWXIbNQXG0MoaTyogKBHVOI2AHdBi1Je+x8pPVdrGryKwaeSxEEE\nn6JuOga+VpjsNQLxdZ3BDSgi20NlIxpOzf7DhI0ADWwtqa0L4WeoIUn871oC+S2e\nU58Zw1KxGc/ZKmyM9z8xNsPeU1Jr2qQ0kOvc9j9Q00ywy5RpXqgRkGQ6VXlBBgyv\niPUEFVvY0ehHkfisFQ==\n-----END CERTIFICATE-----\n";
String device_id;

/*
 * PIN and General Instance
 * ------------------------
 * PIR        : [13]
 * Flash LED  : [4]
 */
#define pinPIR 12
#define pinFlashLed 4

/*
 * Camera Instance
 * ---------------
 * Format : SPIFFS
 */
#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

/**
 * General Board Functions
 * -----------------------
 */

void setFlashLED(bool state) {
  digitalWrite(pinFlashLed, state);
}

void LEDFlashBlink(int blink_count, int time_delay) {
  setFlashLED(LOW);
  for (int i = 1; i <= blink_count * 2; i++) {
    setFlashLED(!digitalRead(pinFlashLed));
    delay(time_delay);
  }
}

int readStatePIR() {
  return digitalRead(pinPIR);
}

/**
 * Photo and Image Processing Functions
 * ------------------------------------
 */

/* Function to capture the image from ESP32 */
camera_fb_t* captureImage(){
  Serial.println("\nTaking a photo...\n");
  camera_fb_t* rawImage = NULL;
  do {
    rawImage = esp_camera_fb_get();
    delay(100);
    if (!rawImage) {
      Serial.println("Camera capture failed.");
      Serial.println("Carry out the re-capture process...");
    }
  } while (!rawImage);
  Serial.println("Take photo successfully.");
  return rawImage;
}

void capturePhotoSaveSpiffs() {
  /**
  * To handle capture photo, there are 3 steps of handling it
  * - Capture the photo
  * - Format the photo as SPIFFS
  */
  HTTPClient http;
  WiFiClientSecure client;
  bool isValidSPIFFSFormat = false;
  setFlashLED(true);
  camera_fb_t * rawImage = captureImage();
  String imageData = base64::encode((uint8_t*)rawImage->buf, rawImage->len);
  esp_camera_fb_return(rawImage);

  if(WiFi.status() == WL_CONNECTED){
    client.setInsecure(); //skip certificate validation
    http.begin(serverName);
    http.addHeader("Content-Type", "image/jpeg");
    http.addHeader("device_id", device_id);
    int httpResponseCode = http.POST(imageData);
    if(httpResponseCode > 0){
      Serial.println("SUCCESS");
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response); 
    } else{
      Serial.println(httpResponseCode);
      Serial.println("Error on sending POST: ");
      Serial.println(http.errorToString(httpResponseCode));
    }
  }

  http.end();
  Serial.println("\nCapture completed..\n");
  setFlashLED(false);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.println();

  pinMode(pinPIR, INPUT);
  pinMode(pinFlashLed, OUTPUT);

  /* Connect to Wi-Fi */
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    LEDFlashBlink(1, 250);
    Serial.print(".");
  }
  setFlashLED(false);
  Serial.println();
  Serial.print("Successfully connected to ");
  Serial.println(ssid);

  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Camera configuration. */
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_VGA;  //--> FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 20;
  config.fb_count = 2;

  /* Initialize camera */ 
  Serial.println();
  Serial.println("Camera initialization...");
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial.println("ESP32 Cam Restart...");
    ESP.restart();
  }
  Serial.print("Camera initialization was successful.");
  Serial.println();

  // Retrieve MAC Address
  device_id = WiFi.macAddress();
  Serial.println("Device ID : " + device_id);

  /* Loop to stabilize the PIR sensor at first power on.
     --------------------------------------------------
   * I created this loop because from the tests I did that when the PIR sensor starts to turn on,
   * the PIR sensor takes at least 30 seconds to be able to detect movement or objects stably or with little noise.
   * I don't know if it's because of the quality factor of the PIR sensor I have.
   * From this source: https://lastminuteengineers.com/pir-sensor-arduino-tutorial/,
   * indeed the PIR sensor takes 30-60 seconds from the time it is turned on to be able to detect objects or movements properly.
   */
  int cameraStabilizerCount = 2;
  LEDFlashBlink(2, 250);
  Serial.println("Wait 60 seconds for the PIR sensor to stabilize.");
  Serial.println("Count down :");
  for (int i = 59; i > -1; i--) {
    Serial.print(i);
    Serial.println(" second");
    delay(1000);
    if(cameraStabilizerCount > 0){
      captureImage();
      cameraStabilizerCount -= 1;
    }
  }

  Serial.println("The time to stabilize the PIR sensor is complete.\n");
  LEDFlashBlink(2, 1000);
}

void loop() {
  bool isMovementDetected = readStatePIR() == 1 || isStreaming;
  if(isMovementDetected){
    capturePhotoSaveSpiffs();
    delay(5000);
  }
  delay(1);
}