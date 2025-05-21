import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Parameters ---
nx, ny = 100, 50  # Grid size
tau = 0.6  # Relaxation time
omega = 1.0 / tau  # Relaxation parameter
inflow_velocity = 0.1
UPDATES = 10
STEPS = 500000

# --- D2Q9 Lattice ---
directions = np.array(
    [
        [0, 0],
        [1, 0],
        [0, 1],
        [-1, 0],
        [0, -1],
        [1, 1],
        [-1, 1],
        [-1, -1],
        [1, -1],
    ]
)
weights = np.array([4 / 9] + [1 / 9] * 4 + [1 / 36] * 4)
opp = [0, 3, 4, 1, 2, 7, 8, 5, 6]  # Opposite directions

# --- Initialization ---
f = np.zeros((9, nx, ny))
rho = np.ones((nx, ny))
ux = np.zeros((nx, ny))
uy = np.zeros((nx, ny))


def equilibrium(i, rho, ux, uy):
    cu = 3 * (directions[i, 0] * ux + directions[i, 1] * uy)
    usqr = 1.5 * (ux**2 + uy**2)
    return weights[i] * rho * (1 + cu + 0.5 * cu**2 - usqr)


# Initialize with equilibrium
for i in range(9):
    f[i] = equilibrium(i, rho, ux, uy)

# --- Matplotlib Setup ---
fig, ax = plt.subplots()
im = ax.imshow(
    np.sqrt(ux**2 + uy**2).T, cmap="plasma", origin="lower", vmin=0, vmax=0.15
)
plt.colorbar(im, ax=ax)
ax.set_title("LBM Fluid Simulation (Velocity Magnitude)")


# --- Main Simulation Step ---
def lbm_step():
    global f, rho, ux, uy

    # Collision
    for i in range(9):
        feq = equilibrium(i, rho, ux, uy)
        f[i] += -omega * (f[i] - feq)

    # Streaming
    for i in range(9):
        dx, dy = directions[i]
        f[i] = np.roll(f[i], dx, axis=0)
        f[i] = np.roll(f[i], dy, axis=1)

    # Bounce-back walls
    # Top and bottom boundaries
    f[:, :, 0] = f[opp, :, 0]  # bottom wall
    f[:, :, -1] = f[opp, :, -1]  # top wall

    # Inflow: left boundary
    rho[0, :] = 1.0
    ux[0, :] = inflow_velocity
    uy[0, :] = 0
    for i in range(9):
        f[i, 0, :] = equilibrium(i, rho[0, :], ux[0, :], uy[0, :])

    # Recompute macroscopic variables
    rho = np.sum(f, axis=0)
    ux = np.sum(f * directions[:, 0].reshape(9, 1, 1), axis=0) / rho
    uy = np.sum(f * directions[:, 1].reshape(9, 1, 1), axis=0) / rho


# --- Animation Function ---
def update(frame):
    for _ in range(UPDATES):  # More LBM steps per frame
        lbm_step()
    speed = np.sqrt(ux**2 + uy**2)
    im.set_array(speed.T)
    ax.set_title(f"Step {frame * UPDATES}")
    return [im]


ani = animation.FuncAnimation(
    fig, update, frames=STEPS, interval=1, blit=False
)
# ani.save("animation.gif", writer="imagemagick", fps=15)
plt.show()
