# EKF-Application-and-Simulation-in-SLAM-problem

This project applies the Extended Kalman Filter Simultaneous Localization and Mapping (EKF-SLAM) algorithm in MATLAB. The algorithm is used for estimating the state of a robot while simultaneously building a map of the environment with landmarks.

## Files

- **main.m**: Main script to run the EKF-SLAM simulation.
- **initialize_robot_and_landmarks.m**: Initializes the robot's state, the landmarks, and other parameters required for the simulation.
- **simulate_robot_motion.m**: Simulates the robot's motion in the environment while displaying the robot's trajectory and landmarks.
- **ekf_slam.m**: Implements the EKF-SLAM algorithm to update the robot's state and covariance matrix based on noisy measurements.
- **wrapToPi.m**: Utility function to normalize angles to the range \([-π, π]\).

## Usage

1. **Initialization**: The robot and landmarks are initialized with predefined states and positions.

2. **Simulation**:
   - The robot's motion is simulated over a period of 20 seconds with defined linear and angular velocities.
   - Noise is added to both the linear and angular speeds to simulate real-world conditions.

3. **EKF-SLAM Execution**:
   - The robot's state and covariance matrix are updated using the EKF-SLAM algorithm at each time step.
   - The robot's trajectory and landmark positions are plotted in real-time.

4. **Final State**: At the end of the simulation, the final state and covariance matrix of the robot are displayed.

## How to Run

1. Clone the repository:
   ```bash
   git clone <repository-url>
   ```
2. Navigate to the project directory and run the `main.m` script in MATLAB.

3. The simulation will display the robot's trajectory and landmark estimations in real-time.

## Parameters

- **Motion Parameters**:
  - `v`: Average linear speed (m/s).
  - `omega`: Average angular speed (rad/s).
  - `T`: Total simulation time (s).

- **Noise Parameters**:
  - `mu_v`: Mean of linear speed (m/s).
  - `sigma_v2`: Variance of linear speed (m/s)^2.
  - `mu_omega`: Mean of angular speed (rad/s).
  - `sigma_omega2`: Variance of angular speed (rad/s)^2.
  - `delta_r2`: Variance of measuring distance (m^2).
  - `delta_phi2`: Variance of measuring angle (rad^2).

## License

This project is licensed under the MIT License.

## Acknowledgements

This project is inspired by standard EKF-SLAM tutorials and materials available in ECE 9156 Topics in Autonomous Robotics.
