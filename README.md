# Quadcopter Simulation

This project provides a Python-based simulation of a quadcopter, modeling its 3D translational (x, y, z) and rotational (yaw ψ, pitch θ, roll φ) dynamics. The simulation uses PID controllers to track desired position setpoints (x, y, z) and yaw angle, with feedback loops to adjust thrust and torques. The code generates response curves to visualize the quadcopter's performance, plotting x, y, z positions in a single graph alongside Euler angles, roll error, and motor dynamics.

## Features

- **3D Dynamics**: Simulates full 3D motion (x, y, z positions) and attitude (roll, pitch, yaw) using a quadcopter model in the plus (+) configuration.
- **PID Control**:
  - Position PID controllers for x, y, z to track setpoints (default: x = 5.0 m, y = 5.0 m, z = 15.0 m).
  - Attitude PID controllers for roll, pitch, yaw to track dynamic setpoints (roll and pitch from position PIDs, yaw fixed at 0°).
  - Non-critically damped height controller for realistic response.
- **Visualization**:
  - Plots x, y, z positions in a single graph with setpoints.
  - Separate plots for roll, pitch, yaw, roll error, and motor angular velocity/acceleration.
  - Output saved as `drone_response.png`.
- **Numerical Stability**:
  - Uses Euler integration with a small time step (dt = 0.005 s).
  - Clamps Euler angles and transformation matrix terms to prevent singularities.
- **Extensibility**: Modular code structure allows for adjusting parameters, setpoints, or adding new controllers.

## Prerequisites

- **Python 3.11 or later**: Recommended for compatibility with dependencies.
- **Dependencies**:
  - `numpy`: For numerical computations.
  - `matplotlib`: For plotting response curves.
- **Operating System**: Tested on macOS (Ventura or later), but should work on Windows and Linux with minor adjustments.
- **Optional**: Virtual environment for isolated dependency management.

## Installation

### 1. Install Python

Ensure Python 3.11 or later is installed. On macOS, use Homebrew for a clean setup:

```bash
brew install python
python3 --version  # Should show Python 3.11.x or later
```

### 2. Set Up a Virtual Environment

Create and activate a virtual environment to manage dependencies:

```bash
python3 -m venv drone_env
source drone_env/bin/activate  # On Windows: drone_env\Scripts\activate
```

### 3. Install Dependencies

Install required Python packages:

```bash
pip install numpy matplotlib
```

### 4. Download the Code

Save the simulation code as `drone_full_control.py` in your working directory. The code is available in this repository or can be copied from the provided source.

## Usage

### Running the Simulation

1. Activate the virtual environment:

   ```bash
   source drone_env/bin/activate
   ```

2. Run the script:

   ```bash
   python3 drone_full_control.py
   ```

3. Check the output:

   - A plot file named `drone_response.png` will be generated in the working directory.
   - The plot includes:
     - **Position Response**: x, y, z positions (m) vs. time (s) in one graph, with setpoints (5.0 m, 5.0 m, 15.0 m).
     - **Roll, Pitch, Yaw**: Euler angles (degrees) vs. time.
     - **Roll Error**: Error between desired and actual roll (degrees).
     - **Motor Dynamics**: Average motor angular velocity (rad/s) and acceleration (rad/s²).

### Customizing Parameters

You can modify the code to adjust:

- **Setpoints**:
  - `setpoint_x`, `setpoint_y`, `setpoint_z`: Desired positions (default: 5.0 m, 5.0 m, 15.0 m).
  - `setpoint_psi`: Desired yaw angle (default: 0.0 rad).
- **PID Gains**:
  - Height PID: `kp_height = 2.0`, `ki_height = 0.1`, `kd_height = 0.5` (non-critically damped).
  - Position PID: `kp_pos = 0.5`, `ki_pos = 0.01`, `kd_pos = 0.2`.
  - Attitude PID: `kp_att = 10.0`, `ki_att = 0.1`, `kd_att = 3.0`.
- **Drone Parameters**:
  - Mass (`m = 2.0 kg`), arm length (`l = 0.25 m`), thrust/torque coefficients (`k_t = 1e-6`, `k_q = 1e-8`), moments of inertia (`I_xx = 0.02`, `I_yy = 0.02`, `I_zz = 0.04`).
- **Simulation Settings**:
  - Time step (`dt = 0.005 s`), total time (`t_max = 100.0 s`).

Edit these in the `drone_full_control.py` file before running.

## Project Structure

- `drone_full_control.py`: Main simulation script containing the quadcopter model, PID controllers, and plotting logic.
- `drone_response.png`: Output plot file (generated after running the script).

## How It Works

- **Dynamics**:
  - **Translational**: Computes x, y, z positions using the thrust vector rotated by Euler angles, accounting for gravity.
  - **Rotational**: Calculates Euler angles (φ, θ, ψ) from torques via body angular velocities (p, q, r) and a transformation matrix.
- **Control**:
  - **Position PID**: X and y controllers generate desired pitch (θ\_d) and roll (φ\_d) to achieve setpoints. Z controller adjusts total thrust.
  - **Attitude PID**: Tracks φ\_d, θ\_d, ψ\_d by computing torques.
- **Output**:
  - Plots show how well the quadcopter tracks setpoints, with x, y, z in one graph for easy comparison.

## Troubleshooting

- **ModuleNotFoundError: No module named 'matplotlib' or 'numpy'**:

  - Ensure dependencies are installed:

    ```bash
    pip install numpy matplotlib
    ```

  - Verify the virtual environment is active.

- **Plot Not Generated**:

  - Check for `drone_response.png` in the working directory.

  - Ensure `matplotlib` backend is compatible. For saving plots, no display is needed, but for interactive plots, install a backend:

    ```bash
    pip install PyQt5
    ```

- **Simulation Divergence**:

  - If positions or angles diverge, reduce `dt` (e.g., to 0.001 s) or adjust PID gains (e.g., lower `kp_pos` to 0.3).

- **Permission Issues**:

  - Fix Homebrew permissions if needed:

    ```bash
    sudo chown -R $(whoami):admin /opt/homebrew
    ```

## Extending the Project

- **Add New Setpoints**: Modify `setpoint_x`, `setpoint_y`, `setpoint_z`, or `setpoint_psi` for different targets.

- **3D Trajectory Plot**: Replace the x, y, z subplot with a 3D plot:

  ```python
  from mpl_toolkits.mplot3d import Axes3D
  ax = fig.add_subplot(projection='3d')
  ax.plot(x, y, z, label='Trajectory')
  ```

- **Advanced Dynamics**: Incorporate wind disturbances or motor delays.

- **Quaternion-Based Attitude**: Replace Euler angles with quaternions for large-angle stability.

## License

This project is provided under the MIT License. See the `LICENSE` file for details (if included).

## Acknowledgments

- Built with Python, `numpy`, and `matplotlib`.
- Inspired by quadcopter control and simulation studies.

For issues or contributions, please open an issue or submit a pull request on the repository (if hosted).
