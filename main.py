import numpy as np
import matplotlib.pyplot as plt

# Drone parameters
m = 2.0  # Mass of the drone (kg)
g = 9.81  # Gravitational acceleration (m/s^2)
k_t = 1e-6  # Thrust coefficient (N/(rad/s)^2)
k_q = 1e-8  # Torque coefficient (Nm/(rad/s)^2)
l = 0.25  # Arm length (m)
n_motors = 4  # Number of motors
I_xx = 0.02  # Moment of inertia about x (kg·m^2)
I_yy = 0.02  # Moment of inertia about y (kg·m^2)
I_zz = 0.04  # Moment of inertia about z (kg·m^2)

# PID controller parameters for height (non-critically damped)
kp_height = 2.0  # Proportional gain
ki_height = 0.1  # Integral gain
kd_height = 0.5  # Derivative gain

# PID controller parameters for position (x, y)
kp_pos = 0.5  # Proportional gain
ki_pos = 0.01  # Integral gain
kd_pos = 0.2  # Derivative gain

# PID controller parameters for attitude (roll, pitch, yaw)
kp_att = 10.0  # Proportional gain
ki_att = 0.1   # Integral gain
kd_att = 3.0   # Derivative gain

# Simulation parameters
dt = 0.005  # Time step (s)
t_max = 100.0  # Total simulation time (s)
t = np.arange(0, t_max, dt)  # Time array
setpoint_x = 5.0  # Desired x position (m)
setpoint_y = 15.0  # Desired y position (m)
setpoint_z = 15.0  # Desired height (m)
setpoint_psi = 0.0  # Desired yaw (rad)

# Initialize arrays
x = np.zeros(len(t))  # X position (m)
y = np.zeros(len(t))  # Y position (m)
z = np.zeros(len(t))  # Height (m)
x_dot = np.zeros(len(t))  # X velocity (m/s)
y_dot = np.zeros(len(t))  # Y velocity (m/s)
z_dot = np.zeros(len(t))  # Vertical velocity (m/s)
phi = np.zeros(len(t))  # Roll angle (rad)
theta = np.zeros(len(t))  # Pitch angle (rad)
psi = np.zeros(len(t))  # Yaw angle (rad)
p = np.zeros(len(t))  # Roll rate (rad/s)
q = np.zeros(len(t))  # Pitch rate (rad/s)
r = np.zeros(len(t))  # Yaw rate (rad/s)
omega = np.zeros(len(t))  # Average motor angular velocity (rad/s)
alpha = np.zeros(len(t))  # Angular acceleration (rad/s^2)
phi_error = np.zeros(len(t))  # Roll error (rad)

# Initialize control variables
error_integral_x = 0.0
error_prev_x = 0.0
error_integral_y = 0.0
error_prev_y = 0.0
error_integral_height = 0.0
error_prev_height = 0.0
error_integral_phi = 0.0
error_prev_phi = 0.0
error_integral_theta = 0.0
error_prev_theta = 0.0
error_integral_psi = 0.0
error_prev_psi = 0.0

# Function to compute rotation matrix (Z-X-Y Euler angles)
def rotation_matrix(psi, theta, phi):
    cpsi, spsi = np.cos(psi), np.sin(psi)
    ctheta, stheta = np.cos(theta), np.sin(theta)
    cphi, sphi = np.cos(phi), np.sin(phi)
    R = np.array([
        [cpsi * ctheta, cpsi * stheta * sphi - spsi * cphi, cpsi * stheta * cphi + spsi * sphi],
        [spsi * ctheta, spsi * stheta * sphi + cpsi * cphi, spsi * stheta * cphi - cpsi * sphi],
        [-stheta, ctheta * sphi, ctheta * cphi]
    ])
    return R

# Function to convert Euler rates to body angular velocities
def euler_to_body_rates(phi, theta):
    ctheta, stheta = np.cos(theta), np.sin(theta)
    cphi, sphi = np.cos(phi), np.sin(phi)
    # Avoid singularity by clamping ctheta
    ctheta = np.clip(ctheta, 0.01, None) if ctheta > 0 else np.clip(ctheta, None, -0.01)
    M = np.array([
        [1, 0, -stheta],
        [0, cphi, ctheta * sphi],
        [0, -sphi, ctheta * cphi]
    ])
    return M

# Function to map control inputs to motor angular velocities
def control_to_motor_omega(F, tau_x, tau_y, tau_z):
    A = np.array([
        [1, 1, 1, 1],
        [0, -l, 0, l],
        [l, 0, -l, 0],
        [-k_q/k_t, k_q/k_t, -k_q/k_t, k_q/k_t]
    ])
    b = np.array([F, tau_x, tau_y, tau_z])
    try:
        f = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        f = np.ones(4) * (F / 4)  # Fallback to equal distribution
    f = np.clip(f, 0, None)
    omega = np.sqrt(f / k_t)
    return omega

# Function to compute torques from motor angular velocities
def motor_omega_to_torques(omega):
    f = k_t * omega**2
    tau_x = l * (f[3] - f[1])  # f_4 - f_2
    tau_y = l * (f[0] - f[2])  # f_1 - f_3
    tau_z = k_q * (omega[1]**2 + omega[3]**2 - omega[0]**2 - omega[2]**2)
    return tau_x, tau_y, tau_z

# Simulation loop
motor_omega = np.zeros(4)  # Initial motor angular velocities
for i in range(1, len(t)):
    # Current states
    x_curr = x[i-1]
    y_curr = y[i-1]
    z_curr = z[i-1]
    x_dot_curr = x_dot[i-1]
    y_dot_curr = y_dot[i-1]
    z_dot_curr = z_dot[i-1]
    phi_curr = phi[i-1]
    theta_curr = theta[i-1]
    psi_curr = psi[i-1]
    p_curr = p[i-1]
    q_curr = q[i-1]
    r_curr = r[i-1]
    
    # Position PID controllers (x, y)
    error_x = setpoint_x - x_curr
    error_integral_x += error_x * dt
    error_derivative_x = (error_x - error_prev_x) / dt
    theta_des = kp_pos * error_x + ki_pos * error_integral_x + kd_pos * error_derivative_x
    theta_des = np.clip(theta_des, -np.pi/4, np.pi/4)  # Limit desired pitch
    
    error_y = setpoint_y - y_curr
    error_integral_y += error_y * dt
    error_derivative_y = (error_y - error_prev_y) / dt
    phi_des = -(kp_pos * error_y + ki_pos * error_integral_y + kd_pos * error_derivative_y)  # Negative for roll
    phi_des = np.clip(phi_des, -np.pi/4, np.pi/4)  # Limit desired roll
    
    # Height PID controller
    error_height = setpoint_z - z_curr
    error_integral_height += error_height * dt
    error_derivative_height = (error_height - error_prev_height) / dt
    u_height = kp_height * error_height + ki_height * error_integral_height + kd_height * error_derivative_height
    total_thrust = m * g + u_height
    total_thrust = max(total_thrust, 0)
    
    # Attitude PID controllers
    error_phi = phi_des - phi_curr
    phi_error[i] = error_phi
    error_integral_phi += error_phi * dt
    error_derivative_phi = (error_phi - error_prev_phi) / dt
    tau_x_des = kp_att * error_phi + ki_att * error_integral_phi + kd_att * error_derivative_phi
    
    error_theta = theta_des - theta_curr
    error_integral_theta += error_theta * dt
    error_derivative_theta = (error_theta - error_prev_theta) / dt
    tau_y_des = kp_att * error_theta + ki_att * error_integral_theta + kd_att * error_derivative_theta
    
    error_psi = setpoint_psi - psi_curr
    error_integral_psi += error_psi * dt
    error_derivative_psi = (error_psi - error_prev_psi) / dt
    tau_z_des = kp_att * error_psi + ki_att * error_integral_psi + kd_att * error_derivative_psi
    
    # Map desired thrust and torques to motor angular velocities
    motor_omega = control_to_motor_omega(total_thrust, tau_x_des, tau_y_des, tau_z_des)
    omega[i] = np.mean(motor_omega)
    alpha[i] = (omega[i] - omega[i-1]) / dt
    
    # Compute actual torques from motor angular velocities
    tau_x, tau_y, tau_z = motor_omega_to_torques(motor_omega)
    
    # Translational dynamics (full 3D)
    R = rotation_matrix(psi_curr, theta_curr, phi_curr)
    thrust_vector = R @ np.array([0, 0, total_thrust])
    a = (thrust_vector - np.array([0, 0, m * g])) / m
    x_dot[i] = x_dot_curr + a[0] * dt
    y_dot[i] = y_dot_curr + a[1] * dt
    z_dot[i] = z_dot_curr + a[2] * dt
    x[i] = x_curr + x_dot_curr * dt
    y[i] = y_curr + y_dot_curr * dt
    z[i] = z_curr + z_dot_curr * dt
    
    # Rotational dynamics
    omega_b = np.array([p_curr, q_curr, r_curr])
    I = np.diag([I_xx, I_yy, I_zz])
    tau = np.array([tau_x, tau_y, tau_z])
    omega_b_dot = np.linalg.inv(I) @ (tau - np.cross(omega_b, I @ omega_b))
    p[i] = p_curr + omega_b_dot[0] * dt
    q[i] = q_curr + omega_b_dot[1] * dt
    r[i] = r_curr + omega_b_dot[2] * dt
    
    # Update Euler angles
    M = euler_to_body_rates(phi_curr, theta_curr)
    euler_rates = M @ np.array([p[i], q[i], r[i]])
    phi[i] = phi_curr + euler_rates[0] * dt
    theta[i] = theta_curr + euler_rates[1] * dt
    psi[i] = psi_curr + euler_rates[2] * dt
    
    # Clamp angles to prevent numerical divergence
    phi[i] = np.clip(phi[i], -np.pi/2, np.pi/2)
    theta[i] = np.clip(theta[i], -np.pi/2, np.pi/2)
    
    # Update previous errors
    error_prev_x = error_x
    error_prev_y = error_y
    error_prev_height = error_height
    error_prev_phi = error_phi
    error_prev_theta = error_theta
    error_prev_psi = error_psi

# Plotting the response curves
fig, (ax1, ax4, ax5, ax6, ax7, ax8) = plt.subplots(6, 1, figsize=(10, 32))

# # X position response
# ax1.plot(t, x, label='Actual X')
# ax1.axhline(y=setpoint_x, color='r', linestyle='--', label='Setpoint')
# ax1.set_title('Drone X Position Response Curve')
# ax1.set_xlabel('Time (s)')
# ax1.set_ylabel('X Position (m)')
# ax1.grid(True)
# ax1.legend()

# # Y position response
# ax2.plot(t, y, label='Actual Y')
# ax2.axhline(y=setpoint_y, color='r', linestyle='--', label='Setpoint')
# ax2.set_title('Drone Y Position Response Curve')
# ax2.set_xlabel('Time (s)')
# ax2.set_ylabel('Y Position (m)')
# ax2.grid(True)
# ax2.legend()

# # Height response
# ax3.plot(t, z, label='Actual Height')
# ax3.axhline(y=setpoint_z, color='r', linestyle='--', label='Setpoint')
# ax3.set_title('Drone Height Response Curve')
# ax3.set_xlabel('Time (s)')
# ax3.set_ylabel('Height (m)')
# ax3.grid(True)
# ax3.legend()


# X, Y, Z position response (same graph)
ax1.plot(t, x, label='Actual X', color='b')
ax1.plot(t, y, label='Actual Y', color='g')
ax1.plot(t, z, label='Actual Z', color='r')
ax1.axhline(y=setpoint_x, color='b', linestyle='--', label='X Setpoint')
ax1.axhline(y=setpoint_y, color='g', linestyle='--', label='Y Setpoint')
ax1.axhline(y=setpoint_z, color='r', linestyle='--', label='Z Setpoint')
ax1.set_title('Drone Position Response Curves (X, Y, Z)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Position (m)')
ax1.grid(True)
ax1.legend()

# Roll response
ax4.plot(t, np.degrees(phi), label='Actual Roll', color='b')
ax4.set_title('Drone Roll Response Curve')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Roll (deg)')
ax4.grid(True)
ax4.legend()

# Pitch response
ax5.plot(t, np.degrees(theta), label='Actual Pitch', color='g')
ax5.set_title('Drone Pitch Response Curve')
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Pitch (deg)')
ax5.grid(True)
ax5.legend()

# Yaw response
ax6.plot(t, np.degrees(psi), label='Actual Yaw', color='m')
ax6.axhline(y=np.degrees(setpoint_psi), color='r', linestyle='--', label='Setpoint')
ax6.set_title('Drone Yaw Response Curve')
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Yaw (deg)')
ax6.grid(True)
ax6.legend()

# Roll error response
ax7.plot(t, np.degrees(phi_error), label='Roll Error', color='orange')
ax7.set_title('Drone Roll Error Response Curve')
ax7.set_xlabel('Time (s)')
ax7.set_ylabel('Error (deg)')
ax7.grid(True)
ax7.legend()

# Angular velocity and acceleration response
ax8.plot(t, omega, label='Motor Angular Velocity', color='c')
ax8.plot(t, alpha, label='Motor Angular Acceleration', color='y')
ax8.set_title('Motor Angular Velocity and Acceleration Response Curves')
ax8.set_xlabel('Time (s)')
ax8.set_ylabel('Value (rad/s, rad/s²)')
ax8.grid(True)
ax8.legend()

plt.tight_layout()
plt.savefig('drone_response.png')