function [state, P, landmarks, dt] = initialize_robot_and_landmarks()
    % Initialize the robot's initial position and orientation
    x0 = 7; % Initial x-coordinate (m)
    y0 = 5; % Initial y-coordinate (m)
    theta0 = pi/2; % Initial orientation (rad)

    % True positions of Landmarks
    landmarks = [1, 7;  % Coordinates of Landmark 1 (x1, y1)
                 10, 8];% Coordinates of Landmark 2 (x2, y2)

    % Sampling period
    dt = 1; % Sampling period (s)

    % Initialize the state vector (considering the position and orientation of the robot)
    state = [x0; y0; theta0; 0; 0; 0; 0]; 

    % Initialize the state covariance matrix, which will depend on the specific design of the system
    P = diag([0.5, 0.5, 0.05, 1, 1, 1, 1]); 

end