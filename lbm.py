import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Parameters ---
nx, ny = 64, 32  # Grid size
tau = 0.6  # Relaxation time
omega = 1.0 / tau  # Relaxation parameter
inflow_velocity = 0.1
UPDATES = 10
STEPS = 100


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
    #np.sqrt(ux**2 + uy**2).T,
    np.sum(f, axis=0).T,
    cmap="plasma", origin="lower", 
)
plt.colorbar(im, ax=ax)
ax.set_title("LBM Fluid Simulation (Velocity Magnitude)")


# --- Main Simulation Step ---
def lbm_step(s):
    global f, rho, ux, uy

    # Collision
    for i in range(9):
        feq = equilibrium(i, rho, ux, uy)
        f[i] += -omega * (f[i] - feq)

    rx, ry = 0, 0
    if s % 100 == 0:
        rx = (np.random.rand() * 2 - 1) * 1.e-1
        ry = (np.random.rand() * 2 - 1) * 1.e-1
            
    force_x = np.sin(2 * np.pi * s/(UPDATES * STEPS)) * -1e-2 + rx
    force_y = -1e-2 + ry # gravity downward

    cs2 = 1.0 / 3.0
    for i in range(9):
        ei = directions[i]
        eu = ei[0]*ux + ei[1]*uy
        force_term = (
            ((ei[0] - ux) + ei[0]*eu / cs2) * force_x +
            ((ei[1] - uy) + ei[1]*eu / cs2) * force_y
        )

        Fi = weights[i] * (1 - 0.5 * omega) * force_term / cs2 * rho
        f[i] += Fi

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
    for i in range(UPDATES):  # More LBM steps per frame
        lbm_step(frame * UPDATES + i)
    speed = np.sqrt(ux**2 + uy**2)
    denst = np.sum(f, axis=0)
    #im.set_array(speed.T)
    im.set_array(denst.T)
    ax.set_title(f"Step {frame * UPDATES}")
    return [im]


ani = animation.FuncAnimation(
    fig, update, frames=STEPS, interval=1, blit=False
)
#ani.save("animation.gif", writer="imagemagick", fps=15)
plt.show()
