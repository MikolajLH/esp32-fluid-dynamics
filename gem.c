void setup_lbm() {
    float initial_rho = 1.0f;
    float initial_ux = 0.0f;
    float initial_uy = 0.0f;

    for (int y = 0; y < NY; ++y) {
        for (int x = 0; x < NX; ++x) {
            // Initialize macroscopic variables
            rho[x][y] = initial_rho;
            ux[x][y] = initial_ux;
            uy[x][y] = initial_uy;

            // Initialize PDFs to equilibrium for fluid at rest
            for (int i = 0; i < 9; ++i) {
                // f_eq for u=(0,0) is simply weights[i] * rho
                f[x][y][i] = weights[i] * initial_rho;
            }
        }
    }
}

void loop_lbm() {
    // --- 1. Collision Step (applied to f, results in f_temp) ---
    for (int y = 0; y < NY; ++y) {
        for (int x = 0; x < NX; ++x) {
            // Calculate macroscopic properties from f[x][y]
            float current_rho = 0.0f;
            float current_ux_num = 0.0f;
            float current_uy_num = 0.0f;

            for (int i = 0; i < 9; ++i) {
                current_rho += f[x][y][i];
                current_ux_num += ex[i] * f[x][y][i];
                current_uy_num += ey[i] * f[x][y][i];
            }
            rho[x][y] = current_rho; // Store for potential use/debug
            float local_ux = (current_rho > 1e-6)? current_ux_num / current_rho : 0.0f;
            float local_uy = (current_rho > 1e-6)? current_uy_num / current_rho : 0.0f;
            ux[x][y] = local_ux; // Store for potential use/debug
            uy[x][y] = local_uy;

            // --- 1.a. Placeholder: Integration of External Forces ---
            // float force_x_gyro = 0.0f; // Obtain from gyroscope
            // float force_y_gyro = 0.0f; // Obtain from gyroscope
            // if (current_rho > 1e-6) { // Avoid division by zero or very small rho
            //     local_ux += TAU * force_x_gyro / current_rho;
            //     local_uy += TAU * force_y_gyro / current_rho;
            // }
            // --- End Placeholder ---

            // Calculate equilibrium distributions (f_eq)
            float u_dot_u = local_ux * local_ux + local_uy * local_uy;
            float f_eq;
            for (int i = 0; i < 9; ++i) {
                float e_dot_u = ex[i] * local_ux + ey[i] * local_uy;
                f_eq[i] = weights[i] * current_rho *
                          (1.0f + 3.0f * e_dot_u + 4.5f * e_dot_u * e_dot_u - 1.5f * u_dot_u);
            }

            // Apply BGK collision (store in f_temp)
            for (int i = 0; i < 9; ++i) {
                f_temp[x][y][i] = f[x][y][i] - (1.0f / TAU) * (f[x][y][i] - f_eq[i]);
            }
        }
    }

    // --- 2. Streaming Step (from f_temp to f) with Bounce-Back Boundary Conditions ---
    for (int y = 0; y < NY; ++y) {
        for (int x = 0; x < NX; ++x) {
            for (int i = 0; i < 9; ++i) {
                int prev_x = x - (int)ex[i];
                int prev_y = y - (int)ey[i];

                // Apply bounce-back for walls (edges of the domain)
                if (prev_x < 0 |
| prev_x >= NX |
| prev_y < 0 |
| prev_y >= NY) {
                    // Particle came from outside (wall), so it reflects.
                    // The f_temp[x][y]] is the particle from (x,y)
                    // that hit the wall in direction OPPOSITE_DIR[i] and is now
                    // returning to (x,y) as f[x][y][i].
                    f[x][y][i] = f_temp[x][y]];
                } else {
                    // Standard streaming from an interior neighbor
                    f[x][y][i] = f_temp[prev_x][prev_y][i];
                }
            }
        }
    }

    // --- 3. (Optional) Update Macroscopic Variables for Display ---
    // This recalculates rho, ux, uy from the new f array for the current timestep's output.
    // The collision step in the *next* iteration will do this anyway.
    // If display logic needs these, calculate them here.
    // For example:
    // for (int y = 0; y < NY; ++y) {
    //     for (int x = 0; x < NX; ++x) {
    //         float current_rho = 0.0f;
    //         //... (recalculate rho[x][y], ux[x][y], uy[x][y] from f[x][y])...
    //     }
    // }

    // --- 4. Placeholder: Updating LED Display ---
    // for (int y = 0; y < NY; ++y) {
    //     for (int x = 0; x < NX; ++x) {
    //         // float value_to_display = rho[x][y]; // or sqrt(ux[x][y]*ux[x][y] + uy[x][y]*uy[x][y])
    //         // set_led_color(x, y, map_value_to_color(value_to_display));
    //     }
    // }
    // --- End Placeholder ---
}