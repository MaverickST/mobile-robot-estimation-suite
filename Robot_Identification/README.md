# Omnidirectional Robot System Identification

## Overview

This project implements a comprehensive **three-stage parameter identification** framework for omnidirectional mobile robots with three omni wheels. The approach combines rigorous kinematic modeling, dynamic simulation, and sequential optimization to estimate motor electrical parameters, robot inertia, and motor compensation factors.

## Robot Configuration

- **Wheel 1** $(w_1)$: 150° - Back-left
- **Wheel 2** $(w_2)$: 270° - Back-right (pointing downward)
- **Wheel 3** $(w_3)$: 30° - Front-right
- **Roller type:** Standard omni wheels (rollers perpendicular to wheel axis)

```
         Y
         ↑
         |
    w₁   |    w₃
    ●    |    ●
         |
---------+--------→ X
         |    
       w₂●
         |
```

## Mathematical Model Development

### 1. Kinematic Model

#### 1.1 Forward Kinematics

For a three-wheeled omnidirectional robot, the relationship between robot velocity in the body frame and wheel angular velocities is given by the **kinematic matrix:**

$$\mathbf{H}$$

$$
\begin{bmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{bmatrix}
= \mathbf{H} \cdot
\begin{bmatrix}
v_x \\
v_y \\
\Omega
\end{bmatrix}
$$

Where:

- Wheel angular velocities [rad/s]: $$\boldsymbol{\omega} = [\omega_1, \omega_2, \omega_3]^T$$

- Robot velocity in body frame [m/s, m/s, rad/s]: $$\mathbf{v} = [v_x, v_y, \Omega]^T$$

- Kinematic transformation matrix: $$\mathbf{H} \in \mathbb{R}^{3 \times 3}$$

#### 1.2 Kinematic Matrix Derivation

For standard omni wheels (rollers perpendicular to wheel axis), each wheel constraint is:

$$
\omega_i = \frac{1}{r}\left[-\sin(\theta_i) \cdot v_x + \cos(\theta_i) \cdot v_y + d \cdot \Omega\right]
$$

Where:

- Angular position of wheel $i$ from robot's X-axis [radians]: $$\theta_i$$

- Wheel radius [m]: $$r$$

- Distance from robot center to wheel contact point [m]: $$d$$

Thus, the kinematic matrix is:

$$
\mathbf{H} = \frac{1}{r}
\begin{bmatrix}
-\sin(\theta_1) & \cos(\theta_1) & d \\
-\sin(\theta_2) & \cos(\theta_2) & d \\
-\sin(\theta_3) & \cos(\theta_3) & d
\end{bmatrix}
$$

For our configuration:

$$\theta_1 = 150\degree = 5\pi/6 \text{ rad}$$

$$\theta_2 = 270\degree = 3\pi/2 \text{ rad}$$

$$\theta_3 = 30\degree = \pi/6 \text{ rad}$$

**Reference:** Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.

#### 1.3 Inverse Kinematics

To obtain robot velocities from wheel speeds:

$$
\begin{bmatrix}
v_x \\
v_y \\
\Omega
\end{bmatrix}
= \mathbf{H}^+ \cdot
\begin{bmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{bmatrix}
$$

Where the Moore-Penrose pseudoinverse is:

$$\mathbf{H}^+ = (\mathbf{H}^T \mathbf{H})^{-1} \mathbf{H}^T$$

### 2. Motor Electrical Model

Each motor follows a DC motor electrical model with back-EMF:

#### 2.1 Voltage Equation

$$
V_i = R_a \cdot i_i + K \cdot \omega_i
$$

Where:

- Applied voltage to motor $i$ [V]: $$V_i$$

- Armature current [A]: $$i_i$$

- Armature resistance [Ω]: $$R_a$$

- Motor constant [V/(rad/s) = Nm/A]: $$K$$

- Wheel angular velocity [rad/s]: $$\omega_i$$

#### 2.2 Torque Equation

Motor torque is proportional to current:

$$
\tau_i = K \cdot i_i
$$

#### 2.3 Combined Motor Model

Solving the voltage equation for current and substituting into torque:

$$
\tau_i = K \cdot \frac{V_i - K \cdot \omega_i}{R_a} = \frac{K}{R_a} \cdot V_i - \frac{K^2}{R_a} \cdot \omega_i
$$

**Reference:** Hughes, A., & Drury, B. (2019). *Electric Motors and Drives: Fundamentals, Types and Applications* (5th ed.). Elsevier.

#### 2.4 PWM to Voltage Conversion

Experimental data from embedded systems uses **PWM duty cycle** (0-100%) instead of voltage. The effective voltage is:

$$
V_{\text{eff}} = \frac{D}{100} \cdot V_{\text{bat}}
$$

This conversion is necessary because the motor's electrical time constant ($\tau_e = L_a/R_a \approx 1$-10 ms) filters the PWM signal (typically 1-20 kHz) into an equivalent DC voltage.

**Important:** $V_{\text{bat}}$ must be measured during data collection as it varies with battery discharge (e.g., 3S LiPo: 12.6V → 9.0V).

**Reference:** Mohan, N., Undeland, T. M., & Robbins, W. P. (2003). *Power Electronics: Converters, Applications, and Design* (3rd ed.). Wiley. Chapter 7.

### 3. Robot Dynamics Model

#### 3.1 Rigid Body Dynamics in Body Frame

The robot's motion is governed by Newton-Euler equations in the body-fixed frame:

$$
\begin{aligned}
M \cdot \mathbf{a}_{\text{local}} &= \mathbf{F}_{\text{local}} \\
I \cdot \alpha &= \tau_z
\end{aligned}
$$

Where:
- $M$: robot mass (kg)
- $I$: moment of inertia about vertical axis (kg·m²)
- $\mathbf{a}_{\text{local}} = [a_x, a_y]^T$: linear acceleration in body frame (m/s²)
- $\alpha$: angular acceleration (rad/s²)
- $\mathbf{F}_{\text{local}} = [F_x, F_y]^T$: total force in body frame (N)
- $\tau_z$: total torque about vertical axis (Nm)

#### 3.2 Force Transformation

Wheel torques are transformed to robot forces using the transpose of the kinematic matrix:

$$
\begin{bmatrix}
F_x \\
F_y \\
\tau_z
\end{bmatrix}
= \frac{1}{r} \cdot \mathbf{H}^T \cdot
\begin{bmatrix}
\tau_1 \\
\tau_2 \\
\tau_3
\end{bmatrix}
$$

This relationship comes from the principle of virtual work and ensures power conservation:

$$
P_{\text{robot}} = \mathbf{v} \cdot \mathbf{F} = \boldsymbol{\omega} \cdot \boldsymbol{\tau}
$$

**Reference:** Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

#### 3.3 Coordinate Transformation

Accelerations must be transformed from body frame to global frame using the rotation matrix:

$$
\mathbf{R}(\phi) =
\begin{bmatrix}
\cos(\phi) & -\sin(\phi) & 0 \\
\sin(\phi) & \cos(\phi) & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

The transformation is:

$$
\mathbf{a}_{\text{global}} = \mathbf{R}(\phi)^T \cdot \mathbf{a}_{\text{local}}
$$

#### 3.4 Coriolis Terms

When transforming between rotating reference frames, Coriolis effects appear:

$$
\begin{aligned}
a_{\text{global},x} &= a_{\text{local},x} + \Omega \cdot v_y \\
a_{\text{global},y} &= a_{\text{local},y} - \Omega \cdot v_x
\end{aligned}
$$

Where $\Omega$ is the angular velocity of the robot.

**Reference:** Goldstein, H., Poole, C., & Safko, J. (2002). *Classical Mechanics* (3rd ed.). Addison-Wesley.

### 4. Complete State-Space Model

#### 4.1 State Vector

$$
\mathbf{x} = [x, y, \phi, v_x, v_y, \Omega]^T
$$

Where:

- Position in global frame [m]: $$x, y$$

- Orientation angle [rad]: $$\phi$$

- Velocity in global frame [m/s]: $$v_x, v_y$$

- Angular velocity [rad/s]: $$\Omega$$

#### 4.2 State-Space Equations

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}, t)
$$

**Expanded form:**

**Position dynamics:**

$$
\dot{x} = v_x, \quad \dot{y} = v_y, \quad \dot{\phi} = \Omega
$$

**Velocity dynamics (with Coriolis terms):**

$$
\dot{v}_x = a_{\text{global},x} + \Omega \cdot v_y
$$

$$
\dot{v}_y = a_{\text{global},y} - \Omega \cdot v_x
$$

**Angular dynamics:**

$$
\dot{\Omega} = \frac{\tau_z}{I}
$$

Where accelerations are computed through the complete chain:

$$
\mathbf{u} \rightarrow \boldsymbol{\tau}_{\text{motor}} \rightarrow \mathbf{F}_{\text{local}} \rightarrow \mathbf{a}_{\text{local}} \rightarrow \mathbf{a}_{\text{global}}
$$

## Three-Stage Identification Protocol

### Stage 1: Motor Electrical Parameters

**Objective:** Estimate armature resistance and motor constant from *steady-state* tests.

$$R_a, K$$

**Experimental Setup:**
1. Lift robot (wheels free to spin) or mount motor on bench
2. Apply constant voltage to one motor: $$V_i$$
3. Wait for steady state (~2 seconds)
4. Measure steady-state wheel speed: $$\omega_i$$
5. Repeat for 5-10 voltage levels

**Physical Model:** At steady state (angular acceleration = 0), motor torque balances friction torque:

$$\dot{\omega} = 0$$

$$
\tau_{\text{motor}} = \tau_{\text{friction}}
$$

With viscous friction model:

$$\tau_{\text{friction}} = b \cdot \omega$$

the electrical equation becomes:

$$
V = R_a \cdot i + K \cdot \omega
$$

And the mechanical equilibrium:

$$
K \cdot i = b \cdot \omega
$$

Solving for current and substituting into voltage equation:

$$i = \frac{b \cdot \omega}{K}$$

$$
V = R_a \cdot \frac{b \cdot \omega}{K} + K \cdot \omega = \left(\frac{R_a \cdot b}{K} + K\right) \cdot \omega
$$

This is a **linear relationship** between $V$ and $\omega$:

$$
\omega = \frac{K}{R_a \cdot b + K^2} \cdot V
$$

**Optimization Problem:**

$$
\min_{R_a, K, b} \sum_{i=1}^{N} \left(\omega_{\text{meas},i} - \frac{K \cdot V_i}{R_a \cdot b + K^2}\right)^2
$$

**Output:**

- Armature resistance [Ω]: $$R_a$$
- Motor constant [V/(rad/s)]: $$K$$  
- Friction coefficient [Nm/(rad/s)]: $$b$$

### Stage 2: Moment of Inertia

**Objective:** Estimate robot inertia from pure rotation maneuvers.

$$I$$

**Experimental Setup:**
1. Command robot to rotate in place (minimal translation):
   $$v_x \approx 0, \quad v_y \approx 0$$
2. Vary angular velocity: slow → fast → stop → reverse
3. Record orientation and angular velocity for 10-15 seconds:
   $$\phi(t), \quad \Omega(t)$$
4. Ensure minimal translation (wheels don't slip)

**Why This Works:** Decouples rotational dynamics from translational. The rotational equation:

$$
I \cdot \dot{\Omega} = \tau_z
$$

depends only on $I$ and is less affected by friction/mass uncertainties.

**Optimization Problem:** Minimize weighted error in angular states:

$$
\min_{I} \sum_{t} \left[w_\phi \cdot (\phi_{\text{meas}}(t) - \phi_{\text{model}}(t))^2 + w_\Omega \cdot (\Omega_{\text{meas}}(t) - \Omega_{\text{model}}(t))^2\right]
$$

Where:

- Weight for orientation tracking: $$w_\phi = 10.0$$

- Weight for angular velocity tracking: $$w_\Omega = 5.0$$

- Translation states are ignored (weights = 0)

**Output:**

- Moment of inertia [kg·m²]: $$I$$

### Stage 3: Motor Compensation Factors

**Objective:** Account for motor-to-motor variations due to manufacturing tolerances.

$$C_{u_1}, C_{u_2}, C_{u_3}$$

**Experimental Setup:**
1. Execute rich trajectory (e.g., square path: right → up → left → down)
2. Include both translations and rotations
3. Record complete state for 15-20 seconds:
   $$\mathbf{x}(t)$$
4. Use varied speeds and directions

**Why This is Needed:** Even with identical motor models, real motors have variations:
- Slight differences in winding resistance
- Bearing friction asymmetries
- Magnetic field non-uniformities
- Assembly tolerances

**Compensation Model:** Scale commanded voltages individually:

$$
u_{\text{actual},i} = C_{u_i} \cdot u_{\text{commanded},i}, \quad i = 1, 2, 3
$$

**Optimization Problem:** Minimize position and orientation tracking errors:

$$
\min_{C_{u_1}, C_{u_2}, C_{u_3}} \sum_{t} \left[w_{\text{pos}} \cdot \|\mathbf{r}_{\text{meas}}(t) - \mathbf{r}_{\text{model}}(t)\|^2 + w_\phi \cdot (\phi_{\text{meas}}(t) - \phi_{\text{model}}(t))^2\right]
$$

Where:

- Weight for position tracking (x, y): $$w_{\text{pos}} = 3.0$$

- Weight for orientation tracking: $$w_\phi = 2.0$$

- Velocity weights (lower priority): $$w_v = 0.5$$

**Output:**

- Compensation factors [dimensionless, nominal = 1.0]: $$C_{u_1}, C_{u_2}, C_{u_3}$$

**Typical Values:** 0.85 - 1.15 (within ±15% indicates acceptable motor matching)

## Installation

### Requirements
- Python 3.8+
- NumPy ≥ 1.21.0
- SciPy ≥ 1.7.0
- Matplotlib ≥ 3.4.0
- scikit-learn ≥ 0.24.0

### Setup (Windows PowerShell)

```powershell
# Create virtual environment
python -m venv venv

# Activate
.\venv\Scripts\Activate.ps1

# Install dependencies
pip install -r requirements.txt
```

### Setup (Linux/macOS)

```bash
# Create virtual environment
python3 -m venv venv

# Activate
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## Usage

### Running Examples

The project includes multiple examples for different use cases:

#### 1. Main Example (Synthetic Data)
```powershell
.\run.ps1
# or
.\run.ps1 -Example main
```

Runs the complete three-stage identification with synthetic data. Good for:
- Understanding the identification workflow
- Testing the implementation
- Verifying the code setup

#### 2. Synthetic Data Examples
```powershell
.\run.ps1 -Example synthetic
```

Runs detailed examples including:
- Kinematic validation with visualization
- Complete three-stage identification
- Multiple test scenarios

#### 3. Stage 1 Experimental Identification
```powershell
.\run.ps1 -Example stage1
```

Processes **real experimental data** from `data/raw/stage1.txt` for motor parameter identification:
- Loads encoder and PWM data
- Applies Kalman filtering (Q=0.001, R=1.12)
- Converts PWM to voltage (6S battery, 22.2V nominal)
- Extracts steady-state values from 8 segments
- Identifies Ra, K parameters for selected motor
- Generates comprehensive plots

**Configuration:** Edit `src/examples/stage1_experimental.py` to:
- Change `MOTOR_ID` (1, 2, or 3)
- Adjust battery voltage if needed
- Modify segment processing parameters

### Expected Outputs

**Console Output:**
- **Stage 1:** Motor parameters with goodness-of-fit
  - $$R_a, K, R^2$$
- **Stage 2:** Inertia with angular tracking metrics
  - $$I, \text{RMSE}, R^2$$
- **Stage 3:** Compensation factors with full state validation
  - $$C_{u_1}, C_{u_2}, C_{u_3}$$

**Plots Generated:**
1. **Wheel Geometry:** Visual verification of wheel positions and orientations
2. **Motor Characterization:** Voltage vs. speed curve with fitted model
3. **Angular State Tracking:** Orientation and angular velocity comparison (Stage 2)
4. **2D Trajectory:** Measured vs. model trajectories in X-Y plane (Stage 3)
5. **State-by-State Validation:** All 6 states over time with RMSE metrics

### Using Experimental Data (Custom Integration)

To use your own experimental data, replace the synthetic data generation with your measurements:

```python
from main import ThreeStageIdentification

# Robot parameters (measure these physically)
wheel_angles = [150, 270, 30]  # degrees
d = 0.099  # distance to wheels (m)
r = 0.0325  # wheel radius (m)
M = 3.178  # robot mass (kg)

# Initialize identifier
identifier = ThreeStageIdentification(wheel_angles, d, r, M)

# Stage 1: Load motor bench test data
# pwm_vals: array of PWM duty cycles (0-100%)
# speeds: array of steady-state wheel speeds (rad/s)
# V_battery: battery voltage during test (V)
from main import pwm_to_voltage
voltages = pwm_to_voltage(pwm_vals, V_battery)
Ra, K = identifier.stage1_motor_parameters(voltages, speeds, motor_id=1)

# Stage 2: Load rotation trajectory data
# t_rot: time vector (s)
# u_rot: control inputs (N x 3 array, in volts)
# y_rot: measured states (N x 6 array: x, y, phi, vx, vy, omega)
I = identifier.stage2_inertia(t_rot, u_rot, y_rot, Ra, K, plot=True)

# Stage 3: Load full trajectory data
# t_full: time vector (s)
# u_full: control inputs (N x 3 array, in volts)
# y_full: measured states (N x 6 array)
Cu1, Cu2, Cu3 = identifier.stage3_compensation(
    t_full, u_full, y_full, Ra, K, I, plot=True
)

# Display summary
identifier.get_summary()
```

### PWM to Voltage Conversion

If your embedded system uses PWM (0-100%) instead of voltages:

```python
from main import pwm_to_voltage

# Example: Convert PWM array to voltages
pwm_data = [65.0, 70.0, 60.0]  # PWM percentages for motors 1, 2, 3
V_battery = 11.8  # Measured battery voltage during experiment (V)

# Convert to voltages
u_voltages = pwm_to_voltage(pwm_data, V_battery)
# Result: [7.67, 8.26, 7.08] volts

# For time-series data:
import numpy as np
pwm_trajectory = np.array([[50, 50, 50],
                           [60, 55, 58],
                           [70, 65, 68]])
V_bat = 12.0
u_trajectory = pwm_to_voltage(pwm_trajectory, V_bat)
```

**Important:** Always log $V_{\text{bat}}$ during experiments as it decreases with discharge!

## Project Structure

```
Robot Identification/
├── main.py                         # Entry point - runs examples
├── src/                            # Main package
│   ├── kinematics/                # Kinematic modeling
│   │   └── validator.py           # KinematicValidator class
│   ├── models/                    # Robot dynamics
│   │   └── robot.py               # OmnidirectionalRobot class
│   ├── identification/            # Identification algorithms
│   │   └── three_stage.py         # ThreeStageIdentification class
│   ├── utils/                     # Utilities (PWM conversion, etc.)
│   └── examples/                  # Usage examples
├── requirements.txt               # Python dependencies
└── STRUCTURE.md                   # Detailed architecture docs
```

**Usage:**
```python
# Run complete example workflow
python main.py

# Or import modules for custom usage
from src.kinematics import KinematicValidator
from src.models import OmnidirectionalRobot
from src.identification import ThreeStageIdentification
```

See `STRUCTURE.md` for detailed module documentation and development guidelines.

## Future Work

- [ ] Load experimental data from files (CSV/MAT format)
- [ ] Real-time data acquisition interface
- [ ] Adaptive compensation for time-varying parameters
- [ ] Extended Kalman Filter for online estimation
- [ ] GUI for parameter tuning and visualization
- [ ] Export identified models to standard formats (URDF, JSON)

## Theoretical Background and References

### Omnidirectional Kinematics
1. **Lynch, K. M., & Park, F. C.** (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press. Chapter 13: Wheeled Mobile Robots.

2. **Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D.** (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press. Chapter 3: Mobile Robot Kinematics.

3. **Campion, G., Bastin, G., & D'Andréa-Novel, B.** (1996). Structural properties and classification of kinematic and dynamic models of wheeled mobile robots. *IEEE Transactions on Robotics and Automation*, 12(1), 47-62.

### DC Motor Modeling
4. **Hughes, A., & Drury, B.** (2019). *Electric Motors and Drives: Fundamentals, Types and Applications* (5th ed.). Elsevier.

5. **Chapman, S. J.** (2012). *Electric Machinery Fundamentals* (5th ed.). McGraw-Hill.

### System Identification
6. **Ljung, L.** (1999). *System Identification: Theory for the User* (2nd ed.). Prentice Hall. Chapters on nonlinear system identification.

7. **Åström, K. J., & Murray, R. M.** (2008). *Feedback Systems: An Introduction for Scientists and Engineers*. Princeton University Press.

### Rigid Body Dynamics
8. **Goldstein, H., Poole, C., & Safko, J.** (2002). *Classical Mechanics* (3rd ed.). Addison-Wesley. Chapter 4: The Kinematics of Rigid Body Motion.

9. **Murray, R. M., Li, Z., & Sastry, S. S.** (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.

## License

This project is intended for academic and research purposes.

## Author

Implementation with validated kinematics and three-stage identification protocol.
