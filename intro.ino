void ball_setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.printf("\n\nstarting...\n");

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

  Serial.printf("setup exit\n");
}

void ball_update(float dt) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  static float et = 0.f;
  static float timer = 0.f;
  et += dt;
  timer += dt;

  // gravity
  static const float grav = 25.f;

  // ball position and radius
  static float pos_x = 10.f;
  static float pos_y = 15.f;
  static const int16_t r = 5;

  // ball's velocity
  static float vx = 0.f;
  static float vy = 0.f;

  float acc_x = -a.acceleration.z;
  float acc_y = a.acceleration.y;

  const float acc_length =
      std::sqrtf(acc_x * acc_x + acc_y * acc_y);
  acc_x /= acc_length;
  acc_y /= acc_length;

  // motion equations
  vx += grav * acc_x * dt;
  vy += grav * acc_y * dt;
  pos_x += vx * dt;
  pos_y += vy * dt;

  // bouncing
  if (int16_t(pos_x) + r >= DISP_RES_X ||
      int16_t(pos_x) - r < 0) {
    vx = -vx * 0.999f;
  }

  if (int16_t(pos_y) + r >= DISP_RES_Y ||
      int16_t(pos_y) - r < 0) {
    vy = -vy * 0.999f;
  }
  //

  // clamping
  pos_x = std::max(float(0 + r), pos_x);
  pos_x = std::min(float(DISP_RES_X - r), pos_x);

  pos_y = std::max(float(0 + r), pos_y);
  pos_y = std::min(float(DISP_RES_Y - r), pos_y);
  //

  // logging
  if (timer >= 1.f) {
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");
    Serial.printf(
        "dt - %f\np - (%f, %f)\nv - (%f, %f)\n\n",
        dt,
        pos_x,
        pos_y,
        vx,
        vy
    );
    Serial.printf("a - (%f, %f)\n", acc_x, acc_y);
    timer = 0.f;
  }

  //*
  // ball drawing
  const uint16_t bg_color = disp_color(0, 0, 0x77);
  const uint16_t cr_color = disp_color(0, 0xff, 0);
  dma_display->fillScreen(bg_color);
  // dma_display->clearScreen();
  dma_display->fillCircle(
      int16_t(pos_x), int16_t(pos_y), r, cr_color
  );
  //*/

  /*
  for(int col = 0; col < DISP_RES_X; col++){
    float part_r = sinf(float(col) * et * 0.5);
    float part_g = cos(float(col) * et * 0.3);

    uint16_t row_color = disp_color(uint8_t(part_r * part_r *
  255.f), uint8_t(part_g * part_g * 255.f), 0); for(int row = 0;
  row < DISP_RES_Y; row++){ dma_display->drawPixel(col, row,
  row_color);
    }
  }
  //*/
}

void ball_loop() {
  static unsigned long end = millis();
  static unsigned long beg = millis();
  float dt = float(end - beg) / 1000.f;
  ball_update(dt);
  vTaskDelay(1);
  beg = end;
  end = millis();
}
