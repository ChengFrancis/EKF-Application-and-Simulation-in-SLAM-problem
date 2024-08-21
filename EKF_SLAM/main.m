% Main script to run EKF-SLAM

% Initialize the robot and landmarks
[state, P, landmarks, dt] = initialize_robot_and_landmarks();

% Define the robot's motion parameters
v = 2; % Average linear speed (m/s)
omega = 1; % Average angular speed (rad/s)
T = 20; % Total simulation time (s)

% Initialize noise parameters
mu_v = 2; % Mean of linear speed (m/s)
sigma_v2 = 0.2; % Variance of linear speed (m/s)^2
mu_omega = 1; % Mean of angular speed (rad/s)
sigma_omega2 = 0.1; % Variance of angular speed (rad/s)^2
delta_r2 = 0.1; % Variance of measuring distance (m^2)
delta_phi2 = 0.1; % Variance of measuring angle (rad^2)

% Run simulation
simulate_robot_motion(state, P, dt, T, mu_v, sigma_v2, mu_omega, sigma_omega2, landmarks);

% Execute EKF SLAM
for t = 0:dt:T
    % Run ekf_slam to update state and P
    [state, P] = ekf_slam(state, P, dt, landmarks, mu_v, sigma_v2, mu_omega, sigma_omega2, delta_r2, delta_phi2);
    
    drawnow;
end

% Print the final state and covariance matrix
disp('Final state:');
disp(state);
disp('Final covariance matrix:');
disp(P);