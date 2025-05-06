function plotBladeSurface(xy, r, chord, twist)
% plotBladeSurface - Plots a 3D blade surface with blade along X-axis
%
% Inputs:
%   xy    - Nx2 array of airfoil coordinates [x, z] (chordwise vs. thickness)
%   r     - 1xM vector of radial positions (blade span, will be along X)
%   chord - 1xM vector of chord lengths at each radial station
%   twist - 1xM vector of twist angles (in degrees) at each station

    % Validate inputs
    if size(xy,2) ~= 2
        error('xy must be an Nx2 array');
    end
    if length(r) ~= length(chord) || length(r) ~= length(twist)
        error('r, chord, and twist must be the same length');
    end

    % Number of airfoil points and radial stations
    nPoints = size(xy,1);
    nStations = length(r);

    % Initialize 3D coordinate arrays
    X = zeros(nPoints, nStations); % spanwise (blade length)
    Y = zeros(nPoints, nStations); % chordwise direction
    Z = zeros(nPoints, nStations); % vertical (thickness)

    for i = 1:nStations
        % Scale airfoil shape by chord
        airfoil = xy * chord(i);

        % Twist rotation around X-axis (blade axis)
        theta = deg2rad(twist(i));
        rotationMatrix = [1 0 0; 
                          0 cos(theta) -sin(theta); 
                          0 sin(theta)  cos(theta)];

        % Convert to 3D (add X = 0 initially), then rotate
        pts3D = [zeros(nPoints,1), airfoil(:,1), airfoil(:,2)]; % [X, Y, Z]
        rotated = (rotationMatrix * pts3D')';

        % Assign rotated points
        X(:,i) = r(i);         % constant X for this radial station
        Y(:,i) = rotated(:,2); % chordwise (Y)
        Z(:,i) = rotated(:,3); % vertical (Z)
    end

    % Plot blade surface
    surf(X, Y, Z, 'FaceColor', 'interp', 'EdgeColor', 'k', 'FaceAlpha', 0.9);
    xlabel('Blade Length (X)');
    ylabel('Chordwise (Y)');
    zlabel('Thickness (Z)');
    title('3D Blade Surface');
    axis equal;
    grid on;
    view(3);

end
