function simulate_robot_motion(state, P, dt, T, mu_v, sigma_v2, mu_omega, sigma_omega2, landmarks)
    % Initialize the figure
    figure;
    hold on;
    xlim([-5, 15]);
    ylim([-5, 15]);
    xlabel('X');
    ylabel('Y');
    title('Robot Trajectory and Landmark Estimation');
    grid on;

    % Plot Landmarks
    scatter(landmarks(:,1), landmarks(:,2), 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0 .7 .7]);
    text(landmarks(:,1), landmarks(:,2), {'Landmark 1', 'Landmark 2'}, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    
    % Initialize storage for the path
    pathX = [];
    pathY = [];
    
    % Main loop
    for t = 0:dt:T
        % Update the robot's position here (with noise)
        v_noisy = normrnd(mu_v, sqrt(sigma_v2));
        omega_noisy = normrnd(mu_omega, sqrt(sigma_omega2));
        
        % Update the position
        x = state(1) + v_noisy * cos(state(3)) * dt;
        y = state(2) + v_noisy * sin(state(3)) * dt;
        theta = wrapToPi(state(3) + omega_noisy * dt);
        
        % Update the state vector
        state(1) = x;
        state(2) = y;
        state(3) = theta;

        % Save the robot's path
        pathX(end+1) = x;
        pathY(end+1) = y;
        
        % Draw the current robot position
        plot(pathX, pathY, 'b.-');
        scatter(x, y, 'filled', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
        
        % Pause for a moment to observe the dynamic effect
        pause(0.1);
    end
    
    hold off;
end