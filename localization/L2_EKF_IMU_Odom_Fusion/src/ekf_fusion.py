import numpy as np
import matplotlib.pyplot as plt

def simulate_motion(T=20.0, dt=0.1):
    """
    Simulate a simple differential-drive robot moving in a curve.
    Returns time, true_states, odom_measurements, imu_measurements.
    State = [x, y, theta].
    """
    steps = int(T / dt)
    t = np.linspace(0, T, steps)

    # Commanded controls (v, omega)
    v_cmd = 1.0      # m/s constant forward
    omega_cmd = 0.3  # rad/s constant turn

    true_states = np.zeros((steps, 3))
    odom_meas = np.zeros_like(true_states)
    imu_meas = np.zeros(steps)  # yaw angle from IMU (with drift + noise)

    # Noise parameters
    odom_pos_sigma = 0.2            # m
    odom_theta_sigma = np.deg2rad(5.0)
    imu_theta_sigma = np.deg2rad(2.0)
    imu_drift_per_sec = np.deg2rad(0.5)

    theta_imu = 0.0
    drift = 0.0

    for k in range(1, steps):
        x, y, th = true_states[k-1]

        # ---- True motion ----
        x += v_cmd * np.cos(th) * dt
        y += v_cmd * np.sin(th) * dt
        th += omega_cmd * dt
        # wrap to [-pi, pi]
        th = (th + np.pi) % (2 * np.pi) - np.pi

        true_states[k] = [x, y, th]

        # ---- Odometry: noisy pose ----
        odom_meas[k, 0] = x + np.random.randn() * odom_pos_sigma
        odom_meas[k, 1] = y + np.random.randn() * odom_pos_sigma
        odom_meas[k, 2] = th + np.random.randn() * odom_theta_sigma

        # ---- IMU: yaw with drift + noise ----
        drift += imu_drift_per_sec * dt
        theta_imu = th + drift + np.random.randn() * imu_theta_sigma
        imu_meas[k] = theta_imu

    return t, true_states, odom_meas, imu_meas, v_cmd, omega_cmd, dt


def ekf_fusion(t, odom_meas, imu_meas, v_cmd, omega_cmd, dt):
    """
    EKF with state x = [x, y, theta].

    Process model (prediction):
        x_k+1 = x_k + v * cos(theta) * dt
        y_k+1 = y_k + v * sin(theta) * dt
        θ_k+1 = θ_k + ω * dt

    Measurement model:
        z = [x_odom, y_odom, theta_imu]^T

    So h(x) = [x, y, theta]^T  (identity mapping).
    """
    steps = len(t)
    x_est = np.zeros((steps, 3))

    # Initial guess from first odometry reading
    x_est[0] = odom_meas[0]

    # Initial covariance
    P = np.diag([0.5**2, 0.5**2, np.deg2rad(10.0)**2])

    # Process noise covariance (model uncertainty)
    q_pos = 0.05
    q_theta = np.deg2rad(1.0)
    Q = np.diag([q_pos**2, q_pos**2, q_theta**2])

    # Measurement noise covariance (sensor noise)
    r_odom_pos = 0.2
    r_imu_theta = np.deg2rad(2.0)
    R = np.diag([r_odom_pos**2, r_odom_pos**2, r_imu_theta**2])

    I = np.eye(3)

    for k in range(1, steps):
        x, y, th = x_est[k-1]

        # ---------- PREDICTION STEP ----------
        # f(x_k, u_k)
        x_pred = np.zeros(3)
        x_pred[0] = x + v_cmd * np.cos(th) * dt
        x_pred[1] = y + v_cmd * np.sin(th) * dt
        x_pred[2] = th + omega_cmd * dt
        x_pred[2] = (x_pred[2] + np.pi) % (2 * np.pi) - np.pi

        # Jacobian F = ∂f/∂x
        F = np.array([
            [1.0, 0.0, -v_cmd * np.sin(th) * dt],
            [0.0, 1.0,  v_cmd * np.cos(th) * dt],
            [0.0, 0.0, 1.0]
        ])

        P_pred = F @ P @ F.T + Q

        # ---------- UPDATE STEP ----------
        # Measurement vector: odom x,y + IMU theta
        z = np.array([
            odom_meas[k, 0],
            odom_meas[k, 1],
            imu_meas[k]
        ])

        # h(x) = [x, y, theta]^T
        h = np.array([
            x_pred[0],
            x_pred[1],
            x_pred[2]
        ])

        # Jacobian H = ∂h/∂x = I for this simple model
        H = np.eye(3)

        # Innovation / residual
        y_res = z - h
        # wrap residual angle to [-pi, pi]
        y_res[2] = (y_res[2] + np.pi) % (2 * np.pi) - np.pi

        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)     # Kalman gain

        x_upd = x_pred + K @ y_res
        x_upd[2] = (x_upd[2] + np.pi) % (2 * np.pi) - np.pi

        P = (I - K @ H) @ P_pred

        x_est[k] = x_upd

    return x_est


def main():
    t, true_states, odom_meas, imu_meas, v_cmd, omega_cmd, dt = simulate_motion()
    x_est = ekf_fusion(t, odom_meas, imu_meas, v_cmd, omega_cmd, dt)

    # ---------- Plot trajectories ----------
    plt.figure()
    plt.plot(true_states[:, 0], true_states[:, 1], label="True")
    plt.plot(odom_meas[:, 0], odom_meas[:, 1], label="Odometry", alpha=0.5)
    plt.plot(x_est[:, 0], x_est[:, 1], label="EKF", linestyle="--")
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("EKF Fusion of IMU + Odometry")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
