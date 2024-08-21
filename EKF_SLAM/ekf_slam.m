function [state, P] = ekf_slam(state, P, dt, landmarks, mu_v, sigma_v2, mu_omega, sigma_omega2, delta_r2, delta_phi2)
    % Extract the robot's current pose
    x = state(1);
    y = state(2);
    theta = state(3);
    
    % Predict the robot's next pose with noise
    v_noisy = normrnd(mu_v, sqrt(sigma_v2));
    omega_noisy = normrnd(mu_omega, sqrt(sigma_omega2));
    x_pred = x + v_noisy * cos(theta) * dt;
    y_pred = y + v_noisy * sin(theta) * dt;
    theta_pred = theta + omega_noisy * dt;
    theta_pred = wrapToPi(theta_pred); % Keep theta within -pi to pi

    % Dimension of the state vector
    state_dim = length(state); 

    % Prediction update for state and covariance
    state_pred = [x_pred; y_pred; theta_pred; zeros(state_dim-3, 1)]; 
    Fx = eye(state_dim); 
    Fx(1:3, 1:3) = [1 0 -v_noisy * sin(theta) * dt;
                    0 1 v_noisy * cos(theta) * dt;
                    0 0 1]; 
    
    % Update the predicted covariance
    Q = zeros(state_dim); 
    Q(1:3, 1:3) = diag([sigma_v2, sigma_v2, sigma_omega2]);
    P_pred = Fx * P * Fx' + Q; 
    
    % For each landmark
    for i = 1:size(landmarks, 1)
        % Calculate expected measurement
        lx = landmarks(i, 1);
        ly = landmarks(i, 2);
        range_exp = sqrt((lx - x_pred)^2 + (ly - y_pred)^2);
        bearing_exp = atan2(ly - y_pred, lx - x_pred) - theta_pred;
        bearing_exp = wrapToPi(bearing_exp); % Keep the bearing within -pi to pi

        % Calculate the Jacobian matrix for the measurement model
        H = zeros(2, length(state)); % Initialize the H matrix
        dx = lx - x_pred;
        dy = ly - y_pred;
        q = dx^2 + dy^2; 

        H(:, 1:3) = [ -dx/sqrt(q), -dy/sqrt(q), 0;
                      dy/q,         -dx/q,      -1]; % Update the first three states (robot's x, y, and theta)
    
        % Calculate the innovation covariance
        R = [delta_r2, 0;
             0, delta_phi2];
        S = H * P_pred * H' + R;
    
        % Calculate the Kalman gain
        K = P_pred * H' / S;
        
        % Simulate actual measurements
        z_actual_range = range_exp + normrnd(0, sqrt(delta_r2));  % Simulated measurement distance
        z_actual_bearing = bearing_exp + normrnd(0, sqrt(delta_phi2));  % Simulated measurement angle
        z_actual_bearing = wrapToPi(z_actual_bearing);  % Ensure the angle is within -pi to pi

        % Assemble z_actual
        z_actual = [z_actual_range; z_actual_bearing];

        % Calculate the expected measurement z_exp
        z_exp = [range_exp; bearing_exp];

        % Calculate the innovation
        innovation = z_actual - z_exp;
        innovation(2) = wrapToPi(innovation(2)); % Bearing should be wrapped to [-pi pi]
    
        % Update the state with the innovation
        state_pred = state_pred + K * innovation;
        
        % Update the covariance matrix
        P_pred = (eye(length(state)) - K * H) * P_pred;
        
        % Store updated state back in state variable
        state(1) = state_pred(1);
        state(2) = state_pred(2);
        state(3) = wrapToPi(state_pred(3)); % Ensure theta is within [-pi pi]

    end

    % Output the updated state and covariance
    state = state_pred;
    P = P_pred;
end