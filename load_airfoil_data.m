function [alpha, Cl, Cd] = load_airfoil_data(filename)
    %LOAD_AIRFOIL_DATA Load angle of attack, Cl, and Cd from airfoil data file.
    %
    %   [alpha, Cl, Cd] = LOAD_AIRFOIL_DATA(filename) reads the specified
    %   text file, skipping the first 12 header lines, and returns vectors
    %   of angle of attack (alpha), lift coefficient (Cl), and drag coefficient (Cd).
    
    % Open the file
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open the file: %s', filename);
    end

    % Skip the first 12 lines (headers)
    for i = 1:12
        fgetl(fid);
    end

    % Read the numeric data from the rest of the file
    data = fscanf(fid, '%f  %f  %f  %f  %f  %f  %f', [7, Inf]);
    fclose(fid);
    data = data';

    % Extract the relevant columns
    alpha = data(:,1); % Angle of attack (deg)
    Cl    = data(:,2); % Lift coefficient
    Cd    = data(:,3); % Drag coefficient
end
