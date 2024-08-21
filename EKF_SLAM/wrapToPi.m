function theta = wrapToPi(theta)
    % Wraps angle in radians to [-pi pi]
    theta = mod(theta + pi, 2 * pi) - pi;
end