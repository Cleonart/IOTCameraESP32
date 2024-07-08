#include "esp_camera.h"
#include "SPI.h"
#include "driver/rtc_io.h"
#include "ESP32_MailClient.h"
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>

/*
 * Network Instance
 * ----------------
 * SSID      : SET BY USER
 * Password  : SET BY USER
 */
const char* ssid = "OPPO A9 2020";
const char* password = "12345678";

/*
 * PIN and General Instance
 * ------------------------
 * PIR        : [13]
 * Flash LED  : [4]
 */
#define pinPIR 13
#define pinFlashLed 4

/*
 * Mail Server Instance
 * -----------------------------
 * Hostname : smtp.gmail.com
 * Port     : 465 (SSL)
 */
#define emailSenderAccount "iotprojek23@gmail.com"
#define emailSenderName "ESP32-CAM IoT Projek"
#define emailSenderAppPassword "iyfp iwdl ajgc tkdr"
#define smtpServer "smtp.gmail.com"
#define smtpServerPort 465
#define emailSubject "ESP32-CAM Photo Captured"
#define emailRecipient "kristiandame@gmail.com"
SMTPData smtpData;

/*
 * Camera Instance
 * ---------------
 * Format : SPIFFS
 */
#define FILE_PHOTO "/photo.jpg" 
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
  setFlashLED(LOW)
  for (int i = 1; i <= blink_count * 2; i++) {
    setFlashLED(!digitalRead(pinFlashLed))
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

/* Subroutine for checking if SPIFFS is a valid SPIFFS */
bool isValidSPIFFS(fs::FS& fs) {
  File image = fs.open(FILE_PHOTO);
  unsigned int imageSize = image.size();
  Serial.printf("File name: %s | size: %d\n", FILE_PHOTO, imageSize);
  image.close();
  return imageSize > 100
}

/* Subroutine for formatting SPIFFS */
void formatSPIFFS() {
  bool formatted = SPIFFS.format();
  Serial.println("\nFormat SPIFFS...");
  if (formatted) {
    Serial.println("\n\nSuccess formatting");
    return
  }
  Serial.println("\n\nError formatting\n");
}

/* Function to capture the image from ESP32 */
camera_fb_t* captureImage(){
  Serial.println("\nTaking a photo...\n");
  camera_fb_t* rawImage = NULL;
  do {
    setFlashLED(true);
    rawImage = esp_camera_fb_get();
    delay(2000);
    if (!rawImage) {
      Serial.println("Camera capture failed.");
      Serial.println("Carry out the re-capture process...");
    }
    setFlashLED(false);
  } while (!rawImage);
  Serial.println("Take photo successfully.");
  return rawImage
}

void capturePhotoSaveSpiffs() {
  /**
  * To handle capture photo, there are 3 steps of handling it
  * - Capture the photo
  * - Format the photo as SPIFFS
  */
  bool isValidSPIFFSFormat = false;
  camera_fb_t* rawImage = captureImage();

  do {
    LEDFlashBlink(2, 250);

    /* Photo file name */
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file in writing mode.");
      formatSPIFFS();
      continue;
    }

    /* Begin the write to SPIFFS format */
    file.write(rawImage->buf, rawImage->len);
    Serial.print("The picture has been saved in ");
    Serial.print(FILE_PHOTO);
    Serial.print(" - Size: ");
    Serial.print(file.size());
    Serial.println(" bytes.");
    file.close();

    /* Check if file has been correctly saved in SPIFFS */
    Serial.println("Checking if the picture file has been saved correctly in SPIFFS...");
    isValidSPIFFSFormat = isValidSPIFFS(SPIFFS);
    if (isValidSPIFFSFormat == 1) {
      Serial.println("The picture file has been saved correctly in SPIFFS.");
    } else {
      Serial.println("The picture file is not saved correctly in SPIFFS.");
      Serial.println("Carry out the re-save process...");
      Serial.println();
    }
  } while (!isValidSPIFFSFormat);

  // Clean and return the frame buffer back to the driver for reuse.
  esp_camera_fb_return(rawImage); 
  LEDFlashBlink(1, 1000);
  Serial.println("\nCapture completed..\n");
}

// Callback function to get the Email sending status
void sendCallback(SendStatus msg) {
  Serial.println(msg.info());  //--> Print the current status
}

void sendPhoto() {
  LEDFlashBlink(3, 250);
  Serial.println("Sending email...");

  // Set the SMTP Server Email host, port, account and password
  smtpData.setLogin(
    smtpServer,
    smtpServerPort,
    emailSenderAccount,
    emailSenderAppPassword);

  smtpData.setSender(emailSenderName, emailSenderAccount);
  smtpData.setPriority("High");
  smtpData.setSubject(emailSubject);
  smtpData.setMessage("<h2>Photo captured with ESP32-CAM and attached in this email.</h2>", true);
  smtpData.addRecipient(emailRecipient);
  smtpData.addAttachFile(FILE_PHOTO, "image/jpg");
  smtpData.setFileStorageType(MailClientStorageType::SPIFFS);
  smtpData.setSendCallback(sendCallback);
  if (!MailClient.sendMail(smtpData))
    Serial.println("Error sending Email, " + MailClient.smtpErrorReason());

  // Clear all data from Email object to free memory
  smtpData.empty();
  LEDFlashBlink(1, 1000);
  delay(2000);
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

  /* Starting to mount SPIFFS */
  Serial.println("Starting to mount SPIFFS...");
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    Serial.println("ESP32 Cam Restart...");
    ESP.restart();
  } else {
    Serial.println("SPIFFS mounted successfully");
  }

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

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;  //--> FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    /*
     * From source https://randomnerdtutorials.com/esp32-cam-ov2640-camera-settings/ :
     * - The image quality (jpeg_quality) can be a number between 0 and 63.
     * - Higher numbers mean lower quality.
     * - Lower numbers mean higher quality.
     * - Very low numbers for image quality, specially at higher resolution can make the ESP32-CAM to crash or it may not be able to take the photos properly.
     */
    config.jpeg_quality = 20;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

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

  /* Loop to stabilize the PIR sensor at first power on.
     --------------------------------------------------
   * I created this loop because from the tests I did that when the PIR sensor starts to turn on,
   * the PIR sensor takes at least 30 seconds to be able to detect movement or objects stably or with little noise.
   * I don't know if it's because of the quality factor of the PIR sensor I have.
   * From this source: https://lastminuteengineers.com/pir-sensor-arduino-tutorial/,
   * indeed the PIR sensor takes 30-60 seconds from the time it is turned on to be able to detect objects or movements properly.
   */
  int cameraStabilizerCount = 3
  LEDFlashBlink(2, 250);
  Serial.println("Wait 60 seconds for the PIR sensor to stabilize.");
  Serial.println("Count down :");
  for (int i = 59; i > -1; i--) {
    Serial.print(i);
    Serial.println(" second");
    delay(1000);
    if(cameraStabilizerCount > 0){
      captureImage()
      cameraStabilizerCount -= 1
    }
  }

  Serial.println("The time to stabilize the PIR sensor is complete.\n");
  LEDFlashBlink(2, 1000);
}

void loop() {
  bool isMovementDetected = readStatePIR() == 1 if (isMovementDetected) {
    capturePhotoSaveSpiffs();
    sendPhoto();
  }
  delay(1);
}
