import numpy as np
import matplotlib.pyplot as plt

L1 = 15.5
L2 = 12
L3 = 12
C = 8.5

def deg2rad(d):
    return d * np.pi / 180.0

def rad2deg(r):
    return r * 180.0 / np.pi
def plot_solutions(l1, l2, l3, C_offset, solutions, x_target, y_target, phi_target_rad):
    fig, ax = plt.subplots(figsize=(8, 8))
    colors = ['blue', 'green']
    for i, (t1, t2, t3) in enumerate(solutions):
        j0_x, j0_y = 0, C_offset
        j1_x = l1 * np.cos(t1)
        j1_y = l1 * np.sin(t1) + C_offset
        j2_x = j1_x + l2 * np.cos(t1 + t2)
        j2_y = j1_y + l2 * np.sin(t1 + t2)
        j3_x = j2_x + l3 * np.cos(t1 + t2 + t3)
        j3_y = j2_y + l3 * np.sin(t1 + t2 + t3)

        plot_x = [j0_x, j1_x, j2_x, j3_x]
        plot_y = [j0_y, j1_y, j2_y, j3_y]

        t1_deg = rad2deg(t1)
        t2_deg = rad2deg(t2)
        t3_deg = rad2deg(t3)

        ax.plot(plot_x, plot_y, 'o-',
                label=f'Sol {i + 1} (t1,t2,t3: {t1_deg:.1f}°, {t2_deg:.1f}°, {t3_deg:.1f}°)',
                color=colors[i],
                linewidth=3,
                markersize=8)

    ax.plot(0, 0, 'k+', markersize=10, label='World (0,0)')
    ax.plot(0, C_offset, 'ko', markersize=10, label='Robot Base (0, C)')
    ax.plot(x_target, y_target, 'rx', markersize=15,
            mew=3, label=f'Target (x,y: {x_target}, {y_target})')

    ax.arrow(x_target, y_target,
             0.5 * l3 * np.cos(phi_target_rad),
             0.5 * l3 * np.sin(phi_target_rad),
             head_width=1.0, head_length=1.5, fc='r', ec='r')

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title(f"3R IK Solutions (L1={L1}, L2={L2}, L3={L3}, C={C_offset})")
    ax.axis('equal')  # This is critical for kinematics!
    ax.grid(True)
    ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()
    plt.show()

