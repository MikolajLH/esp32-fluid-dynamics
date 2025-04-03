#include <freertos/FreeRTOS.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSerif9pt7b.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <Wire.h>
#include "FastIMU.h"

#define DISP_128X64 0
#define DISP_64X32 1

#if DISP_128X64
#define DISP_RES_X 128  // width in pixels of each panel module.
#define DISP_RES_Y 64   // height in pixels of each panel module.
#elif DISP_64X32
#define DISP_RES_X 64  // width in pixels of each panel module.
#define DISP_RES_Y 32  // height in pixels of each panel module.
#endif

#define DISP_CHAIN 1  // number of HUB75 panels chained one to another

//here we define the display interface pins
//please remember about the E pin for larger HUB75 displays
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
  E_PIN = -1,  //in this case pin E is left unused
#endif
  LAT_PIN = 4,
  OE_PIN = 15,
  CLK_PIN = 16
};

MatrixPanel_I2S_DMA* dma_display = nullptr;


//Convert RGB888 to RGB565 which is used by the library
//It gets converted anyway inside the libs,
//but this is just to keep things compatible
inline uint16_t disp_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

//modify to suit your taste :)
void show_splash_screen(void) {
  dma_display->clearScreen();
  dma_display->setFont(&FreeSerif9pt7b);
  dma_display->setCursor(2, 20);
  dma_display->setTextColor(disp_color(0xFF, 0x00, 0xFF));
  dma_display->print("TEST");
  vTaskDelay(2000);
}

//a general-purpose hex-dump function for simple serial port debugging
void dump(void* mem, uint16_t len) {
  Serial.printf("dump of address %08X, len=%d, contents:\n", (unsigned int)mem, len);
  uint8_t* membyteptr = (uint8_t*)mem;
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

//https://how2electronics.com/esp32-with-bmi160-accelerometer-gyroscope-sensor/
/*
const int sda_pin = 21;     // I2C SDA Pin for ESP32 (default GPIO 21)
const int scl_pin = 22;     // I2C SCL Pin for ESP32 (default GPIO 22)
#define IMU_ADDRESS 0x68
BMI160 IMU;
calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;
*/

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.printf("\n\nstarting...\n");
  /*
  //Wire.begin(sda_pin, scl_pin);
  //Wire.begin();

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.printf("Error initializing IMU: ");
    Serial.printf("%d\n", err);
    while (true) {
      ;
    }
  }
  Serial.println("FastIMU calibration & data example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(5000);
  }

  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
  */

  Serial.printf("starting...\n");

  HUB75_I2S_CFG::i2s_pins _pins = { R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN };
  HUB75_I2S_CFG mxconfig(
    DISP_RES_X,  // module width
    DISP_RES_Y,  // module height
    DISP_CHAIN,  // chain length (how many modules are connected in chain)
    _pins        // pin mapping
  );
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(180);  //0-255
  dma_display->clearScreen();

  //Serial.printf("splash screen...\n");
  //show_splash_screen();
  Serial.printf("setup exit\n");
}



void update(float dt){

  /*
  IMU.update();
  IMU.getAccel(&accelData);
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print(accelData.accelZ);
  Serial.print("\t");
  IMU.getGyro(&gyroData);
  Serial.print(gyroData.gyroX);
  Serial.print("\t");
  Serial.print(gyroData.gyroY);
  Serial.print("\t");
  Serial.print(gyroData.gyroZ);
  Serial.println();
  delay(50);
  //*/

  /*
  IMU.update();
  IMU.getGyro(&gyroData);
  float gx = gyroData.gyroX;
  float gy = gyroData.gyroY;
  float gl = sqrtf(gx * gx + gy * gy);
  //*/

  static float et = 0.f;
  static float timer = 0.f;
  et += dt;
  timer += dt;
  

  // gravity
  static const float g = 25.f;

  //ball position and radius
  static float cx = 10.f;
  static float cy = 15.f;
  static const int16_t r = 5;

  // ball's velocity
  static float vx = 0.f;
  static float vy = 0.f;

  //motion equations
  vx += g * dt;
  cx += vx * dt;
  cy += vy * dt;
  
  //clapming
  if(int16_t(cx) + r >= DISP_RES_X){
    vx = -vx;
  }

  //logging
  if(timer >= 1.f){
    Serial.printf("dt - %f\np - (%f, %f)\nv - (%f, %f)\n\n", dt, cx, cy, vx, vy);
    timer = 0.f;
  }

  //*
  //ball drawing
  const uint16_t bg_color = disp_color(0, 0, 0x77);
  const uint16_t cr_color = disp_color(0, 0xff, 0);
  dma_display->fillScreen(bg_color);
  //dma_display->clearScreen();
  dma_display->fillCircle(int16_t(cx), int16_t(cy), r, cr_color);
  //*/

  /*
  for(int col = 0; col < DISP_RES_X; col++){
    float part_r = sinf(float(col) * et * 0.5);
    float part_g = cos(float(col) * et * 0.3);

    uint16_t row_color = disp_color(uint8_t(part_r * part_r * 255.f), uint8_t(part_g * part_g * 255.f), 0);
    for(int row = 0; row < DISP_RES_Y; row++){
      dma_display->drawPixel(col, row, row_color);
    }
  }
  //*/
}

void loop() {
  static unsigned long end = millis();
  static unsigned long beg = millis();
  float dt = float(end - beg) / 1000.f;
  update(dt);
  vTaskDelay(1);
  beg = end;
  end = millis();
}