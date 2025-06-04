// Parameters

#define DIRS 9

// Relaxation time
const float TAU = 0.6;
// Relaxation parameter
const float OMEGA = 1.0 / TAU;
const float INFLOW_VELOCITY = 0.1;
const int16_t UPDATES = 10;
const int16_t STEPS = 100;
const float GRAVITY = 2000;

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
sensors_event_t a, g, temp;

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
      // this function modifies f and f can be used in more
      // advanced arithmetic that would be more logical outside
      // function, but why bother when one can simply use
      // nullptr like that
      if (sub != nullptr) {
        f[j][k] -= sub[j][k];
      }
      // return weights[i] * rho * (1 + cu + 0.5 * cu**2 - usqr)
      f[j][k] = WEIGHTS[i] * rho[j][k] *
                (1 + cu + 0.5 * cu * cu - usqr);
    }
  }
}

// similar to function defined above, for line
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
    f[0][j] = WEIGHTS[i] * rho[0][j] *
              (1 + cu + 0.5 * cu * cu - usqr);
  }
}

void fluid_setup() {
  // Initialize with equilibrium
  for (int16_t i = 0; i < DIRS; i++) {
    equilibrium(f[i], i, rho, ux, uy, nullptr);
  }
}

void fluid_loop() {
  mpu.getEvent(&a, &g, &temp);

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

  force_x = -a.acceleration.z * GRAVITY;
  force_y = a.acceleration.y * GRAVITY;
  // TODO How to calculate gravity properly
  // const float force =
  //     std::sqrtf(force_x * force_x + force_y * force_y);
  // force_x /= force;
  // force_y /= force;

  // cs2 = 1.0 / 3.0
  const float cs2 = 1.0 / 3.0;
  for (int16_t i = 0; i < DIRS; i++) {
    for (int16_t j = 0; j < DISP_RES_X; j++) {
      for (int16_t k = 0; k < DISP_RES_Y; k++) {
        // ei = directions[i]
        const float dx = DIRECTIONS[i][0],
                    dy = DIRECTIONS[i][1];

        // eu = ei[0] * ux + ei[1] * uy
        const float eu = dx * ux[j][k] + dy * uy[j][k];

        // force_term =
        //   ((ei[0] - ux) + ei[0] * eu / cs2) * force_x
        //   + ((ei[1] - uy) + ei[1] * eu / cs2) * force_y
        const float force_term =
            ((dx - ux[j][k]) + dx * eu / cs2) * force_x +
            ((dy - ux[j][k]) + dy * eu / cs2) * force_y;

        // Fi = weights[i] * (1 - 0.5 * omega) *
        //   force_term / cs2 * rho
        // f[i] += Fi
        f[i][j][k] += WEIGHTS[i] * (1 - 0.5 * OMEGA) *
                      force_term / cs2 * rho[j][k];
      }
    }
  }

  // Streaming
  for (int16_t i = 0; i < DIRS; i++) {
    // dx, dy = directions[i]
    const float dx = DIRECTIONS[i][0], dy = DIRECTIONS[i][1];
    // TODO how the fuck
    // f[i] = np.roll(f[i], dx, axis=0)
    // f[i] = np.roll(f[i], dy, axis=1)
  }

  // Bounce-back walls
  // Top and bottom boundaries
  for (int16_t i = 0; i < DIRS; i++) {
    for (int16_t j = 0; j < DISP_RES_X; j++) {
      // f[:, :, 0] = f[opp, :, 0]  # bottom wall
      f[i][j][0] = f[OPPOSITE[i]][j][0];
      // f[:, :, -1] = f[opp, :, -1]  # top wall
      f[i][j][DISP_RES_Y - 1] = f[OPPOSITE[i]][j][DISP_RES_Y];
    }
  }

  // Inflow: left boundary
  for (int16_t i = 0; i < DISP_RES_Y; i++) {
    // rho[0, :] = 1.0
    rho[0][i] = 1.0;
    // ux[0, :] = inflow_velocity
    ux[0][i] = INFLOW_VELOCITY;
    // uy[0, :] = 0
    uy[0][i] = 0;
  }
  for (int16_t i = 0; i < DIRS; i++) {
    // f[i, 0, :] =
    //   equilibrium(i, rho[0, :], ux[0, :], uy[0, :])
    equilibrium0(f[i], i, rho, ux, uy);
  }

  // Recompute macroscopic variables
  for (int16_t j = 0; j < DISP_RES_X; j++) {
    for (int16_t k = 0; k < DISP_RES_Y; k++) {
      for (int16_t i = 0; i < DIRS; i++) {
        // rho = np.sum(f, axis=0)
        rho[j][k] += f[i][j][k];
      }
    }
  }

  for (int16_t j = 0; j < DISP_RES_X; j++) {
    for (int16_t k = 0; k < DISP_RES_Y; k++) {
      for (int16_t i = 0; i < DIRS; i++) {
        // ux = np.sum(
        //   f * directions[:, 0].reshape(9, 1, 1),
        //   axis=0
        // ) / rho

        // uy = np.sum(
        //   f * directions[:, 1].reshape(9, 1, 1),
        //   axis=0
        // ) / rho

        //
      }

      ux[k][j] /= rho[k][j];
      uy[k][j] /= rho[k][j];
    }
  }

  //
}
