#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <Fonts/FreeSerif9pt7b.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>

#define DISP_128X64 0
#define DISP_64X32  1

#if DISP_128X64
#define DISP_RES_X 128 // width in pixels of each panel module.
#define DISP_RES_Y 64  // height in pixels of each panel module.
#elif DISP_64X32
#define DISP_RES_X 64 // width in pixels of each panel module.
#define DISP_RES_Y 32 // height in pixels of each panel module.
#endif

// number of HUB75 panels chained one to another
#define DISP_CHAIN 1

// here we define the display interface pins
// please remember about the E pin for larger HUB75 displays
//(A-E are for selecting scan areas)
enum HubPins {
  R1_PIN = 25,
  G1_PIN = 26,
  B1_PIN = 27,
  R2_PIN = 14,
  G2_PIN = 12,
  B2_PIN = 13,
  A_PIN = 23,
  B_PIN = 19,
  C_PIN = 5,
  D_PIN = 17,
#if DISP_128X64
  E_PIN = 18,
#elif DISP_64X32
  E_PIN = -1, // in this case pin E is left unused
#endif
  LAT_PIN = 4,
  OE_PIN = 15,
  CLK_PIN = 16
};

MatrixPanel_I2S_DMA *dma_display = nullptr;
Adafruit_MPU6050 mpu;

// Convert RGB888 to RGB565 which is used by the library
// It gets converted anyway inside the libs,
// but this is just to keep things compatible
inline uint16_t disp_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// modify to suit your taste :)
void show_splash_screen(void) {
  dma_display->clearScreen();
  dma_display->setFont(&FreeSerif9pt7b);
  dma_display->setCursor(2, 20);
  dma_display->setTextColor(disp_color(0xFF, 0x00, 0xFF));
  dma_display->print("TEST");
  vTaskDelay(2000);
}

// a general-purpose hex-dump function for simple serial port
// debugging
void dump(void *mem, uint16_t len) {
  Serial.printf(
      "dump of address %08X, len=%d, contents:\n",
      (unsigned int)mem,
      len
  );
  uint8_t *membyteptr = (uint8_t *)mem;
  for (int i = 0; i < len; i++) {
    Serial.printf(" %02X", membyteptr[i]);
  }
  Serial.printf("\ndump in text: ");
  for (int i = 0; i < len; i++) {
    char c = membyteptr[i];
    if (isprint(c)) {
      Serial.printf("%c", c);
    }
  }
  Serial.printf("\n");
}
