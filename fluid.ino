// Parameters

#define DIRS 9

// Relaxation time
const float TAU = 0.6;
// Relaxation parameter
const float OMEGA = 1.0 / TAU;
const float INFLOW_VELOCITY = 0.1;
const int16_t UPDATES = 10;
const int16_t STEPS = 100;

// --- D2Q9 Lattice ---
const int16_t DIRECTIONS[DIRS][2] = {
    {0,  0 },
    {1,  0 },
    {0,  1 },
    {-1, 0 },
    {0,  -1},
    {1,  1 },
    {-1, 1 },
    {-1, -1},
    {1,  -1}
};

const float WEIGHTS[DIRS] = {
    4 / 9,
    1 / 9,
    1 / 9,
    1 / 9,
    1 / 9,
    1 / 36,
    1 / 36,
    1 / 36,
    1 / 36
};

// Opposite directions
const int16_t OPPOSITE[DIRS] = {0, 3, 4, 1, 2, 7, 8, 5, 6};

// --- Initialization ---

float f[DIRS][DISP_RES_X][DISP_RES_Y] = {0};
float rho[DISP_RES_X][DISP_RES_Y] = {0};
float ux[DISP_RES_X][DISP_RES_Y] = {0};
float uy[DISP_RES_X][DISP_RES_Y] = {0};
float force_x = 0;
float force_y = 0;

// --- Helpers ---

// f[i] = equilibrium(i, rho, ux, uy)
// def equilibrium(i, rho, ux, uy):
void equilibrium(
    float f[DISP_RES_X][DISP_RES_Y],
    int i,
    float rho[DISP_RES_X][DISP_RES_Y],
    float ux[DISP_RES_X][DISP_RES_Y],
    float uy[DISP_RES_X][DISP_RES_Y],
    float sub[DISP_RES_X][DISP_RES_Y]
) {
  register double cu, usqr;
  for (int16_t j = 0; j < DISP_RES_X; j++) {
    for (int16_t k = 0; k < DISP_RES_Y; k++) {
      // cu = 3 * (directions[i, 0] * ux + directions[i, 1] *
      // uy)
      cu = 3 * DIRECTIONS[i][0] * ux[j][k] +
           DIRECTIONS[i][1] * uy[j][k];
      // usqr = 1.5 * (ux**2 + uy**2)
      usqr = 1.5 * ux[j][k] * ux[j][k] + uy[j][k] * uy[j][k];
      if (sub != nullptr) {
        f[j][k] -= sub[j][k];
      }
      // return weights[i] * rho * (1 + cu + 0.5 * cu**2 - usqr)
      f[j][k] =
          WEIGHTS[i] * rho[j][k] * (1 + cu + 0.5 * cu * cu - usqr);
    }
  }
}

// for line
// f[i, 0, :] = equilibrium(i, rho[0, :], ux[0, :], uy[0, :])
// def equilibrium(i, rho, ux, uy):
void equilibrium0(
    float f[DISP_RES_X][DISP_RES_Y],
    int i,
    float rho[DISP_RES_X][DISP_RES_Y],
    float ux[DISP_RES_X][DISP_RES_Y],
    float uy[DISP_RES_X][DISP_RES_Y]
) {
  register double cu, usqr;
  for (int16_t j = 0; j < DISP_RES_Y; j++) {
    cu = 3 * DIRECTIONS[i][0] * ux[0][j] +
         DIRECTIONS[i][1] * uy[0][j];
    usqr = 1.5 * ux[0][j] * ux[0][j] + uy[0][j] * uy[0][j];
    f[0][j] =
        WEIGHTS[i] * rho[0][j] * (1 + cu + 0.5 * cu * cu - usqr);
  }
}

void fluid_setup() {
  // Initialize with equilibrium
  for (int16_t i = 0; i < DIRS; i++) {
    equilibrium(f[i], i, rho, ux, uy, nullptr);
  }
}

void fluid_loop() {
  // Collision
  for (int16_t i = 0; i < DIRS; i++) {
    // feq = equilibrium(i, rho, ux, uy)
    // f[i] += -omega * (f[i] - feq)
    equilibrium(f[i], i, rho, ux, uy, f[i]);
    for (int16_t j = 0; j < DISP_RES_X; j++) {
      for (int16_t k = 0; k < DISP_RES_Y; k++) {
        // - has been applied in equilibirium function
        f[i][j][k] *= OMEGA;
      }
    }
  }

  //
}
