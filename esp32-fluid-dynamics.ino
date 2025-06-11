#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <Fonts/FreeSerif9pt7b.h>
#include <Wire.h>
#include <math.h>
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
  dma_display->setBrightness8(180); // 0-255
  dma_display->clearScreen();
}



#define SOLVER_ITERATIONS 20
#define NX DISP_RES_X
#define NY DISP_RES_Y
#define SIZE ((NX + 2) * (NY + 2))
#define IX(i, j) ((i) + (NX + 2) * (j))

// (u,v)
static float u[SIZE];
static float v[SIZE];
static float u_prev[SIZE];
static float v_prev[SIZE];

static float dens[SIZE];
static float dens_prev[SIZE]; 


const float diff = 0.0f;    
const float visc = 0.0f;
const float grav_strength = 3.8f;


void add_source(float* x, float* s, float dt)
{
	for (int i=0 ; i < SIZE; ++i)
    x[i] += dt * s[i];
}

void diffuse(int b, float* x, float* x0, float diff, float dt)
{
  //
  for(int i = 0; i < SIZE; ++i) x[i] = 0.f;

	const float a = dt * diff * NX * NY;
  const float z = 1 / (1.f + 4.f*a);
	for (int k=0; k < 20; ++k) {
		for (int i=1; i <= NX; ++i) {
			for (int j=1 ; j <= NY; ++j) {
				x[IX(i, j)] = (x0[IX(i, j)] + a * (x[IX(i - 1, j)] + x[IX(i + 1, j)] + x[IX(i, j - 1)] + x[IX(i, j + 1)])) * z;
			}
		}
		set_bnd(b, x);
	}
}

void advect(int b, float* d, float* d0, float* u, float* v, float dt)
{
  const float dtx = dt * NX;
  const float dty = dt * NY;

	for (int i = 1; i <= NX; i++) {
	  for (int j = 1; j <= NY; j++) {

			float x = i - dtx * u[IX(i, j)];
			if (x < 0.5f) x = 0.5f;
      if (x > NX + 0.5f) x = NX + 0.5f;
      int i0 = (int)x;
      int i1 = i0 + 1;

      float y = j - dty * v[IX(i, j)];
			if (y < 0.5f) y = 0.5f;
      if (y > NY + 0.5f) y = NY + 0.5;
      int j0 = (int)y;
      int j1 = j0 + 1;

			float s1 = x - i0;
      float s0 = 1 - s1;
      float t1 = y - j0;
      float t0 = 1 - t1;

			d[IX(i, j)] = s0 * (t0 * d0[IX(i0, j0)] + t1 * d0[IX(i0, j1)]) + s1 * (t0 * d0[IX(i1, j0)] + t1 * d0[IX(i1, j1)]);
		}
	}
	set_bnd(b, d);
}

void dens_step(float* x, float* x0, float* u, float* v, float diff, float dt)
{
	add_source (x, x0, dt);

  std::swap(x0, x);
	diffuse(0, x, x0, diff, dt);

  std::swap(x0, x);
	advect(0, x, x0, u, v, dt);
}

void vel_step(float* u, float* v, float* u0, float* v0, float visc, float dt)
{
	add_source(u, u0, dt);
  add_source(v, v0, dt);

  std::swap(u0, u);
  std::swap(v0, v);

  diffuse(1, u, u0, visc, dt );
	diffuse(2, v, v0, visc, dt);

	project(u, v, u0, v0);


  std::swap(u0, u);
  std::swap(v0, v);
	
	advect(1, u, u0, u0, v0, dt);
  advect(2, v, v0, u0, v0, dt);

	project(u, v, u0, v0);
}

void project(float* u, float* v, float* p, float* div)
{
	for (int i=1; i <= NX; ++i) {
		for (int j=1; j <= NY; ++j) {
			div[IX(i, j)] = -0.5f * (u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]);
      p[IX(i, j)] = 0.f;
		}
	}
	set_bnd (0, div);
  set_bnd (0, p);

	for (int k=0 ; k < SOLVER_ITERATIONS; k++) {
		for (int i=1; i <= NX; i++) {
			for (int j=1; j <= NY; j++) {
				p[IX(i, j)] = (div[IX(i, j)] + p[IX(i - 1, j)] + p[IX(i + 1, j)] + p[IX(i, j - 1)] + p[IX(i, j + 1)]) / 4.f;
			}
		}
		set_bnd (0, p);
	}

	for (int i=1 ; i <= NX; i++) {
		for (int j=1 ; j <= NY; j++) {
			u[IX(i, j)] -= 0.5 * (p[IX(i + 1, j)] - p[IX(i - 1, j)]);
			v[IX(i, j)] -= 0.5 * (p[IX(i, j + 1)] - p[IX(i, j - 1)]);
		}
	}

	set_bnd(1, u);
  set_bnd(2, v);
}

void set_bnd(int b, float* x)
{
  // x-axis
  if(b == 1) {
    for (int i = 1; i <= NX; i++) {
    x[IX(i, 0)] = -x[IX(i, 1)];
    x[IX(i, NY + 1)] = -x[IX(i, NY)];
    }

    for (int j = 1; j <= NY; j++) {
      x[IX(0, j)] = x[IX(1, j)];
      x[IX(NX + 1, j)] = x[IX(NX, j)];
    }
  }
  
  if(b == 2) {
    for (int i = 1; i <= NX; i++) {
      x[IX(i, 0)] = x[IX(i, 1)];
      x[IX(i, NY + 1)] = x[IX(i, NY)];
    }

    for (int j = 1; j <= NY; j++) {
      x[IX(0, j)] = -x[IX(1, j)];
      x[IX(NX + 1, j)] = -x[IX(NX, j)];
    }
  }

  // density
  if(b == 0) {
    for (int i = 1; i <= NX; i++) {
      x[IX(i, 0)] = x[IX(i, 1)];
      x[IX(i, NY + 1)] = x[IX(i, NY)];
    }

    for (int j = 1; j <= NY; j++) {
      x[IX(0, j)] = x[IX(1, j)];
      x[IX(NX + 1, j)] = x[IX(NX, j)];
    }
  }

  // corners
  x[IX(0, 0)]         = 0.5f * (x[IX(1, 0)] + x[IX(0, 1)]);
  x[IX(0, NY + 1)]   = 0.5f * (x[IX(1, NY + 1)] + x[IX(0, NY)]);
  x[IX(NX + 1, 0)]   = 0.5f * (x[IX(NX, 0)] + x[IX(NX + 1, 1)]);
  x[IX(NX + 1, NY + 1)] = 0.5f * (x[IX(NX, NY + 1)] + x[IX(NX + 1, NY)]);

}

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.printf("\n\nstarting setup...\n");
  Serial.printf("dmu...\n");
  setup_dma();
  Serial.printf("ok...\n");

  Serial.printf("mpu...\n");
  setup_mpu();
  Serial.printf("ok...\n");

   for (int j = 12; j <= 20; j++) {
      for (int i = 28; i <= 40; i++) {
          dens[IX(i,j)] = 100.0f;
      }
  }

    Serial.printf("setup finished\n");
}

void draw_dens(){
  const float max_expected_density = 100.f;
  float md = dens[IX(1, 1)];
  float sm = 0.f;
  for (int i = 1; i <= NX; ++i) {
			for (int j = 1 ; j <= NY; ++j) {
        float raw_density = dens[IX(i, j)];
        if(raw_density > md)md = raw_density;
        sm += raw_density;

        float norm_density = raw_density / max_expected_density;
        if (norm_density > 1.0f) norm_density = 1.0f;
        if (norm_density < 0.0f) norm_density = 0.0f;

        uint8_t gray = (uint8_t)(norm_density * 255.f);

        dma_display->drawPixel(i - 1, j - 1, disp_color(gray, gray, gray));
			}
		}

  float avg = sm / (64.f * 32.f);
  Serial.println(avg);
  Serial.println(md);
  Serial.println("----------");
}


void loop() {
  static unsigned long end = millis();
  static unsigned long beg = millis();
  float dt = float(end - beg) / 1000.f;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //static const float grav = 0.1f;

  float acc_x = -a.acceleration.z;
  float acc_y = a.acceleration.y;
  const float acc_length = std::sqrtf(acc_x * acc_x + acc_y * acc_y);
  acc_x /= acc_length;
  acc_y /= acc_length;

  //acc_x = 0.0f;
  //acc_y = 1.0f;

  for (int i=0 ; i < SIZE; ++i){
    u_prev[i] = grav_strength * acc_x;
    v_prev[i] = grav_strength * acc_y;
    dens_prev[i] = 0.f;
  }

  vel_step (u, v, u_prev, v_prev, visc, dt);
	dens_step(dens, dens_prev, u, v, diff, dt);
	draw_dens();
  vTaskDelay(1);

  beg = end;
  end = millis();
}
