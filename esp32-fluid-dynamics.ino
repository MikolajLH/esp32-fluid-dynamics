#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <Fonts/FreeSerif9pt7b.h>
#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include "Simulation.h"

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
Simulation sim{};

// Convert RGB888 to RGB565 which is used by the library
// It gets converted anyway inside the libs,
// but this is just to keep things compatible
inline uint16_t disp_color(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void setup_mpu() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}
void setup_dma(){
  HUB75_I2S_CFG::i2s_pins _pins = {
      R1_PIN,
      G1_PIN,
      B1_PIN,
      R2_PIN,
      G2_PIN,
      B2_PIN,
      A_PIN,
      B_PIN,
      C_PIN,
      D_PIN,
      E_PIN,
      LAT_PIN,
      OE_PIN,
      CLK_PIN
  };
  HUB75_I2S_CFG mxconfig(
      DISP_RES_X, // module width
      DISP_RES_Y, // module height
      DISP_CHAIN, // chain length (how many modules are
                  // connected in chain)
      _pins       // pin mapping
  );
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(255); // 0-255
  dma_display->clearScreen();
}
const uint8_t viridis_colormap[256][3] = {
    {68, 1, 84},
    {68, 2, 85},
    {68, 3, 87},
    {69, 5, 88},
    {69, 6, 90},
    {69, 8, 91},
    {70, 9, 92},
    {70, 11, 94},
    {70, 12, 95},
    {70, 14, 97},
    {71, 15, 98},
    {71, 17, 99},
    {71, 18, 101},
    {71, 20, 102},
    {71, 21, 103},
    {71, 22, 105},
    {71, 24, 106},
    {72, 25, 107},
    {72, 26, 108},
    {72, 28, 110},
    {72, 29, 111},
    {72, 30, 112},
    {72, 32, 113},
    {72, 33, 114},
    {72, 34, 115},
    {72, 35, 116},
    {71, 37, 117},
    {71, 38, 118},
    {71, 39, 119},
    {71, 40, 120},
    {71, 42, 121},
    {71, 43, 122},
    {71, 44, 123},
    {70, 45, 124},
    {70, 47, 124},
    {70, 48, 125},
    {70, 49, 126},
    {69, 50, 127},
    {69, 52, 127},
    {69, 53, 128},
    {69, 54, 129},
    {68, 55, 129},
    {68, 57, 130},
    {67, 58, 131},
    {67, 59, 131},
    {67, 60, 132},
    {66, 61, 132},
    {66, 62, 133},
    {66, 64, 133},
    {65, 65, 134},
    {65, 66, 134},
    {64, 67, 135},
    {64, 68, 135},
    {63, 69, 135},
    {63, 71, 136},
    {62, 72, 136},
    {62, 73, 137},
    {61, 74, 137},
    {61, 75, 137},
    {61, 76, 137},
    {60, 77, 138},
    {60, 78, 138},
    {59, 80, 138},
    {59, 81, 138},
    {58, 82, 139},
    {58, 83, 139},
    {57, 84, 139},
    {57, 85, 139},
    {56, 86, 139},
    {56, 87, 140},
    {55, 88, 140},
    {55, 89, 140},
    {54, 90, 140},
    {54, 91, 140},
    {53, 92, 140},
    {53, 93, 140},
    {52, 94, 141},
    {52, 95, 141},
    {51, 96, 141},
    {51, 97, 141},
    {50, 98, 141},
    {50, 99, 141},
    {49, 100, 141},
    {49, 101, 141},
    {49, 102, 141},
    {48, 103, 141},
    {48, 104, 141},
    {47, 105, 141},
    {47, 106, 141},
    {46, 107, 142},
    {46, 108, 142},
    {46, 109, 142},
    {45, 110, 142},
    {45, 111, 142},
    {44, 112, 142},
    {44, 113, 142},
    {44, 114, 142},
    {43, 115, 142},
    {43, 116, 142},
    {42, 117, 142},
    {42, 118, 142},
    {42, 119, 142},
    {41, 120, 142},
    {41, 121, 142},
    {40, 122, 142},
    {40, 122, 142},
    {40, 123, 142},
    {39, 124, 142},
    {39, 125, 142},
    {39, 126, 142},
    {38, 127, 142},
    {38, 128, 142},
    {38, 129, 142},
    {37, 130, 142},
    {37, 131, 141},
    {36, 132, 141},
    {36, 133, 141},
    {36, 134, 141},
    {35, 135, 141},
    {35, 136, 141},
    {35, 137, 141},
    {34, 137, 141},
    {34, 138, 141},
    {34, 139, 141},
    {33, 140, 141},
    {33, 141, 140},
    {33, 142, 140},
    {32, 143, 140},
    {32, 144, 140},
    {32, 145, 140},
    {31, 146, 140},
    {31, 147, 139},
    {31, 148, 139},
    {31, 149, 139},
    {31, 150, 139},
    {30, 151, 138},
    {30, 152, 138},
    {30, 153, 138},
    {30, 153, 138},
    {30, 154, 137},
    {30, 155, 137},
    {30, 156, 137},
    {30, 157, 136},
    {30, 158, 136},
    {30, 159, 136},
    {30, 160, 135},
    {31, 161, 135},
    {31, 162, 134},
    {31, 163, 134},
    {32, 164, 133},
    {32, 165, 133},
    {33, 166, 133},
    {33, 167, 132},
    {34, 167, 132},
    {35, 168, 131},
    {35, 169, 130},
    {36, 170, 130},
    {37, 171, 129},
    {38, 172, 129},
    {39, 173, 128},
    {40, 174, 127},
    {41, 175, 127},
    {42, 176, 126},
    {43, 177, 125},
    {44, 177, 125},
    {46, 178, 124},
    {47, 179, 123},
    {48, 180, 122},
    {50, 181, 122},
    {51, 182, 121},
    {53, 183, 120},
    {54, 184, 119},
    {56, 185, 118},
    {57, 185, 118},
    {59, 186, 117},
    {61, 187, 116},
    {62, 188, 115},
    {64, 189, 114},
    {66, 190, 113},
    {68, 190, 112},
    {69, 191, 111},
    {71, 192, 110},
    {73, 193, 109},
    {75, 194, 108},
    {77, 194, 107},
    {79, 195, 105},
    {81, 196, 104},
    {83, 197, 103},
    {85, 198, 102},
    {87, 198, 101},
    {89, 199, 100},
    {91, 200, 98},
    {94, 201, 97},
    {96, 201, 96},
    {98, 202, 95},
    {100, 203, 93},
    {103, 204, 92},
    {105, 204, 91},
    {107, 205, 89},
    {109, 206, 88},
    {112, 206, 86},
    {114, 207, 85},
    {116, 208, 84},
    {119, 208, 82},
    {121, 209, 81},
    {124, 210, 79},
    {126, 210, 78},
    {129, 211, 76},
    {131, 211, 75},
    {134, 212, 73},
    {136, 213, 71},
    {139, 213, 70},
    {141, 214, 68},
    {144, 214, 67},
    {146, 215, 65},
    {149, 215, 63},
    {151, 216, 62},
    {154, 216, 60},
    {157, 217, 58},
    {159, 217, 56},
    {162, 218, 55},
    {165, 218, 53},
    {167, 219, 51},
    {170, 219, 50},
    {173, 220, 48},
    {175, 220, 46},
    {178, 221, 44},
    {181, 221, 43},
    {183, 221, 41},
    {186, 222, 39},
    {189, 222, 38},
    {191, 223, 36},
    {194, 223, 34},
    {197, 223, 33},
    {199, 224, 31},
    {202, 224, 30},
    {205, 224, 29},
    {207, 225, 28},
    {210, 225, 27},
    {212, 225, 26},
    {215, 226, 25},
    {218, 226, 24},
    {220, 226, 24},
    {223, 227, 24},
    {225, 227, 24},
    {228, 227, 24},
    {231, 228, 25},
    {233, 228, 25},
    {236, 228, 26},
    {238, 229, 27},
    {241, 229, 28},
    {243, 229, 30},
    {246, 230, 31},
    {248, 230, 33},
    {250, 230, 34},
    {253, 231, 36},
};

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.printf("\n\nHello:D, starting setup...\n");
  Serial.print("Initial Free Heap (bytes): ");
  Serial.println(ESP.getFreeHeap());

  Serial.print("Largest Free Block (Max Alloc Heap): ");
  Serial.println(ESP.getMaxAllocHeap());

  Serial.printf("dmu...\n");
  setup_dma();
  Serial.printf("ok...\n");

  Serial.printf("mpu...\n");
  setup_mpu();
  Serial.printf("ok...\n");

  Serial.printf("setup finished\n");
  sim.reset();
}

void mapDensityToRGB_Viridis(float density, uint8_t& r, uint8_t& g, uint8_t& b) {
    // Normalize density to an index from 0 to 255
    int index = (int)(density * 255.0f);
    index = std::max(0, std::min(255, index)); // Clamp to be safe

    r = viridis_colormap[index][0];
    g = viridis_colormap[index][1];
    b = viridis_colormap[index][2];
}

void draw_dens(){
  float max_expected_density = 100.f;
  for(int i = 0; i < Simulation::SIZE; ++i)
    if(sim.d[i] > max_expected_density)max_expected_density = sim.d[i];
  
  for (int i = 1; i <= Simulation::Nx; ++i) {
			for (int j = 1; j <= Simulation::Ny; ++j) {
        float dens = sim.d[Simulation::IX(i, j)];
        uint8_t rd = 0;
        uint8_t gr = 0;
        uint8_t bl = 0;

        float norm_density = dens / max_expected_density;
        if (norm_density > 1.0f) norm_density = 1.0f;
        if (norm_density < 0.0f) norm_density = 0.0f;

        mapDensityToRGB_Viridis(norm_density, rd, gr, bl);

        //uint8_t gray = (uint8_t)(norm_density * 255.f);
        
        if(norm_density < 0.01f)
          dma_display->drawPixel(i - 1, j - 1, disp_color(0, 0, 0));
        else{
          dma_display->drawPixel(i - 1, j - 1, disp_color(rd, gr, bl));
        }
			}
		}
}

void loop() {
  static unsigned long end = millis();
  static unsigned long beg = millis();
  float dt = float(end - beg) / 1000.f;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sim.clear_buffer();

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int mx = 0;
    int my = 0;
    int itemsParsed = sscanf(data.c_str(), "%d %d", &mx, &my);
    if(mx >= 0 and mx < DISP_RES_X and my >= 0 and my < DISP_RES_Y){
      sim.d0[Simulation::IX(mx + 1, my + 1)] += 100.f;

      //dma_display->drawPixel(mx, my, disp_color(255, 0, 0));
    }

    if(mx == -1){
      sim.reset();
      dma_display->fillScreen(disp_color(255, 0, 255));
    }
  }

  float acc_x = -a.acceleration.z;
  float acc_y = a.acceleration.y;
  const float acc_length = std::sqrtf(acc_x * acc_x + acc_y * acc_y);
  acc_x /= acc_length;
  acc_y /= acc_length;

  for (int i = 1; i <= Simulation::Nx; ++i)
	  for (int j = 1; j <= Simulation::Ny; ++j)
		  if (sim.d[Simulation::IX(i, j)] > 0.01f)
      {

        sim.vy0[Simulation::IX(i, j)] += acc_y * 10.f;
        sim.vx0[Simulation::IX(i, j)] += acc_x * 10.f;
      }
         


  sim.add_source();
  sim.vort_confinement();
  sim.vel_step();
  sim.den_step();


  // //dma_display->clearScreen();
  draw_dens();

  vTaskDelay(1);

  beg = end;
  end = millis();
}
