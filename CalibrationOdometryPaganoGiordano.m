close all
clear
clc
#import 2d geometry utils
source "tools/utilities/geometry_helpers_2d.m"
disp('loading the matrix');
path = 'dataset.txt';

kinematic_parameters = [0; 0; 0; 0; 0; 0; 0];
max_enc_values = [0; 0];

file = fopen(path, 'r');

if file == -1
    error('Failed to open the file.');
end

#reads text using a cell array of strings
dataset = textscan(file, '%s', 'Delimiter', '\n', 'MultipleDelimsAsOne', 1);
lines = dataset{1}; # array of strings
n_lines = size(lines)(1);

#extracting kinematic parameters
line = lines{3}; # string
line = strsplit(line, ':'){2}; # take strings after ":"
parsed_line = textscan(line, '%f %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1); # cell array of values
kinematic_parameters(1) = parsed_line{1};
kinematic_parameters(2) = parsed_line{2};
kinematic_parameters(3) = parsed_line{3};
kinematic_parameters(4) = parsed_line{4};
line = lines{5}; # string
line = strsplit(line, ':'){2}; # take strings after ":"
parsed_line = textscan(line, '%f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1); # cell array of values
max_enc_values(1) = parsed_line{1};
max_enc_values(2) = parsed_line{2};
line = lines{7};
parts = strsplit(line, ':');
values_str = strtrim(parts{2}); # take strings after ":" and throw away spaces
values_str = strrep(values_str, '[', ''); # remove [
values_str = strrep(values_str, ']', ''); # remove ]
values_str = strrep(values_str, ',', ''); # remove ,
laser_pose = str2num(values_str); # from string of number to numeric array
kinematic_parameters(5) = laser_pose(1);
kinematic_parameters(6) = laser_pose(2);
kinematic_parameters(7) = laser_pose(3);

# after 70 iterations
#kinematic_parameters(1) = 5.5109e-01;
#kinematic_parameters(2) = 9.9767e-03;
#kinematic_parameters(3) = 1.4333e+00;
#kinematic_parameters(4) = -6.6128e-02;
#kinematic_parameters(5) = 1.5791e+00;
#kinematic_parameters(6) = -5.2816e-02;
#kinematic_parameters(7) = 3.0228e-03;

# after 140 iterations
#kinematic_parameters(1) = 5.5030e-01;
#kinematic_parameters(2) = 9.9815e-03;
#kinematic_parameters(3) = 1.4320e+00;
#kinematic_parameters(4) = -6.5843e-02;
#kinematic_parameters(5) = 1.5842e+00;
#kinematic_parameters(6) = -5.2816e-02;
#kinematic_parameters(7) = 3.0228e-03;


disp('Initial Kinematic Parameters'), disp(kinematic_parameters);
disp('Maximal Encoder Values'), disp(max_enc_values);

# Read dataset
function odometry=read_odometry(path)
    file = fopen(path, 'r');

    if file == -1
        error('Failed to open the file.');
    end

    #reads text using a cell array of strings
    dataset = textscan(file, '%s', 'Delimiter', '\n', 'MultipleDelimsAsOne', 1);
    lines = dataset{1}; # array of strings
    n_lines = size(lines)(1);

    #the incremental encoder values are stored in a uint32 variable with max range of:
    max_encoder_value = 4294967295;
    prev_encoder_value = 4294859756;
    half_max_val = 2147429878;

    #odometry matrix, without first 8 lines
    odometry = zeros(n_lines-8, 9);

    #starting from the data
    for line_idx = 9:n_lines
        current_line = lines{line_idx};
        parsed_line = textscan(current_line, 'time: %f ticks: %f %f model_pose: %f %f %f tracker_pose: %f %f %f', 'Delimiter', ' ', 'MultipleDelimsAsOne', 1);
        incremental_tick = parsed_line{3};
        current_encoder_value = incremental_tick;
        tipe_overflow = 0;

        if (current_encoder_value > prev_encoder_value)
            module_tick_difference = current_encoder_value - prev_encoder_value;
            tipe_overflow = 1;
        else
            module_tick_difference = prev_encoder_value - current_encoder_value;
            tipe_overflow = 0;
        endif

        tick_difference = current_encoder_value - prev_encoder_value;

        # Detect overflow conditions
        if (module_tick_difference > half_max_val)
            # disp('Overflow detected');
            if (tipe_overflow == 1)
                incremental_tick = - (max_encoder_value - module_tick_difference);
            else
                incremental_tick = max_encoder_value - module_tick_difference;
            endif
        else
            # disp('No overflow');
            incremental_tick = tick_difference;
        endif

        fid = fopen('incremental_ticks.txt','a');
        fprintf(fid,"%f \n",incremental_tick);
        fclose(fid);

        prev_encoder_value = current_encoder_value;
        odometry(line_idx - 8, :) = [cell2mat(parsed_line(1:2)), incremental_tick, cell2mat(parsed_line(4:9))];
    endfor
endfunction

DATA = read_odometry(path);

function adjusted_dataset = reset_time(DATA)

    # Reinitialize the time column to avoid floating point precision issues
    # Example: eps(1.6e+09) = 2.38e-07
    # Small increments in the last digits would be lost without this adjustment
    time_col = DATA(:,1);
    updated_time = 0;
    previous_timestamp = time_col(1,1);
    num_entries = size(time_col, 1);
    time_col(1,1) = updated_time;

    for entry_idx = 1:(num_entries - 1)
        current_timestamp = time_col(entry_idx + 1, 1);
        time_delta = current_timestamp - previous_timestamp;
        updated_time += time_delta;
        time_col(entry_idx + 1, 1) = updated_time;
        previous_timestamp = current_timestamp;
    endfor

    DATA(:,1) = time_col;
    adjusted_dataset = DATA;

endfunction

DATA = reset_time(DATA);

function pose_T = Dynamic_Front_Tractor_Tricycle( traction_ticks_increment, start_state, steering_ticks, encoder_max_vals, kin_params)

    # Extract kinematic parameters and encoder limits
    k_traction = kin_params(2);
    max_traction = encoder_max_vals(2);
    axis_length = kin_params(3);
    steer_offset = kin_params(4);
    max_steer = encoder_max_vals(1);
    k_steer = kin_params(1);
    theta = start_state(3);

    # Convert traction ticks to meters
    traction_distance = traction_ticks_increment * (k_traction / max_traction);

    # Convert steering ticks to radians, considering positive and negative angles
    if steering_ticks > (max_steer / 2)
        # Negative angles
        steer_angle = - (k_steer * (max_steer - steering_ticks) * 2 * pi / max_steer) + steer_offset;
    else
        # Positive angles
        steer_angle = (k_steer * steering_ticks * 2 * pi / max_steer) + steer_offset;
    endif

    steer_angle

    fid = fopen('steering_angle.txt','a');
    fprintf(fid,"%f \n",steer_angle);
    fclose(fid);

    # Calculate the positional and rotational displacements
    dx = traction_distance * cos(theta + steer_angle);
    dy = traction_distance * sin(theta + steer_angle);
    dtheta = traction_distance * sin(steer_angle) / axis_length;
    # phi coordinate is omitted because we don''t need it

    # Update the state with calculated displacements
    x = start_state(1) + dx;
    y = start_state(2) + dy;
    theta = start_state(3) + dtheta;

    # Final pose of the tricycle
    pose_T = [x; y; theta];

endfunction

function final_laser_pose = calculate_laser_pose(kin_params, odometry_data)

    # Extract kinematic parameters and laser base offsets
    axis_length = kin_params(3);
    laser_offset = [kin_params(5), kin_params(6), kin_params(7)];
    laser_offset_neg = -laser_offset;

    # Transformation matrices for laser positions
    transform_laser_neg = v2t(laser_offset_neg);
    transform_laser = v2t(laser_offset);

    # Determine the number of odometry entries
    num_entries = length(odometry_data(1, :));
    final_laser_pose = zeros(3, num_entries);

    # Compute laser pose for each odometry entry
    for entry_idx = 1:num_entries
        rear_x = odometry_data(1, entry_idx) - axis_length * cos(odometry_data(3, entry_idx));
        rear_y = odometry_data(2, entry_idx) - axis_length * sin(odometry_data(3, entry_idx));
        rear_pose = [rear_x; rear_y; odometry_data(3, entry_idx)];

        transform_rear = v2t(rear_pose);
        transform_laser_pose = transform_rear * transform_laser;
        transformed_laser_neg = transform_laser_neg * transform_laser_pose;

        laser_pose_vector = t2v(transformed_laser_neg);
        final_laser_pose(:, entry_idx) = laser_pose_vector;
    endfor

    # Transpose for consistent output format
    final_laser_pose = final_laser_pose'; # '

endfunction

function Trajectory = final_robot_config(init_state, enc_max_vals, DATA, kin_params, steer_v)
    inc_encoder_data = DATA(:,3);
    abs_encoder_data = DATA(:,2);
    num_steps = size(inc_encoder_data, 1);
    Trajectory = zeros(3, num_steps);
    actual_state = init_state;

    for step_idx = 1:num_steps

        if step_idx == 1    # Initialize the first entry considering initial state and no prior angle or rotational displacement
            steering_value = steer_v; 
        else
            steering_value = abs_encoder_data(step_idx - 1);
        endif

        Trajectory(1:3, step_idx) = Dynamic_Front_Tractor_Tricycle(inc_encoder_data(step_idx), actual_state, steering_value, enc_max_vals, kin_params);
        actual_state = Trajectory(1:3, step_idx);
        
    endfor

endfunction

####################### GROUND TRUTH ##############################

function plot_ground_truth(U, indices)
    # Plot xy ground truth trajectory of Robot

    figure;
    plot(U(:,7), U(:,8)); 
    axis([-5 4 -4 2]);
    title("Ground Truth xy trajectory of Robot");
    pause(5);
endfunction

######################## UNCALIBRATED POSE ###########################

function plot_uncalibrated_pose(initial_state, max_enc_values, U, kinematic_parameters, steer_v)
    # Predicted Uncalibrated Odometry of Laser 

    T = final_robot_config(initial_state, max_enc_values, U, kinematic_parameters, steer_v);
    pose_laser = calculate_laser_pose(kinematic_parameters, T);
    figure;
    plot(pose_laser(:,1), pose_laser(:,2));
    axis([-5 25 -13 2]);
    title("Initial Predicted Uncalibrated xy trajectory of the Robot");
    pause(5);
endfunction

function angle_difference = angle_normalization(theta_1, theta_2)
    # Normalize angles to [-pi, pi]
    norm_theta_1 = mod(theta_1 + pi, 2 * pi) - pi;
    norm_theta_2 = mod(theta_2 + pi, 2 * pi) - pi;
    difference = norm_theta_1 - norm_theta_2;
    angle_difference = mod(difference + pi, 2 * pi) - pi;
endfunction


################### PARAMETER INITIALIZATION  #####################
initial_state = [1.4; 0; 0];
steer_v = 0;
num_kin = length(kinematic_parameters);

#################################### LEAST SQUARES ####################################

function kinematic_parameters = LeastSquares(kinematic_parameters, max_enc_values, initial_state, epsilon, n_iteration, dataset_size, num_kin, steer_v, DATA)

    # Initialize Kernel Parameters
    kernel_threshold = 1; 
    final_threshold = 1e-2; 
    threshold_decay = (kernel_threshold - final_threshold) / n_iteration;

    plot_uncalibrated_pose(initial_state, max_enc_values, DATA, kinematic_parameters, steer_v)

    plot_ground_truth(DATA, 1:size(DATA, 1))

    figure;

    for iteration = 1:n_iteration
        current_threshold = max(kernel_threshold - iteration * threshold_decay, final_threshold);

        # Compute Predicted Values
        final_robot_pose = final_robot_config(initial_state, max_enc_values, DATA, kinematic_parameters, steer_v);
        laser_pred_all = calculate_laser_pose(kinematic_parameters, final_robot_pose);

        laser_plus_all = [];
        laser_minus_all = [];

        # Add Perturbation
        perturbation = zeros(num_kin, 1);

        for i = 1:num_kin
            perturbation(i) = epsilon;

            # Positive perturbation
            front_plus = final_robot_config(initial_state, max_enc_values, DATA, kinematic_parameters + perturbation, steer_v);
            laser_plus = calculate_laser_pose(kinematic_parameters + perturbation, front_plus);

            # Negative perturbation
            front_minus = final_robot_config(initial_state, max_enc_values, DATA, kinematic_parameters - perturbation, steer_v);
            laser_minus = calculate_laser_pose(kinematic_parameters - perturbation, front_minus);

            # Reset perturbation
            perturbation(i) = 0;

            # Stack perturbed datasets
            laser_plus_all = [laser_plus_all; laser_plus];
            laser_minus_all = [laser_minus_all; laser_minus];
        endfor

        #disp('size(laser_plus_all)'), disp(size(laser_plus_all));

        # Initialize Matrices
        delta_x = zeros(num_kin, 1);
        H = zeros(num_kin, num_kin);
        b = zeros(num_kin, 1);
        c = 0;

        for i = 1:dataset_size

            # Compute Error
            error = zeros(3,1);
            pred = laser_pred_all(i, :); 
            meas = DATA(i, 7:9);
            error(1:2) = pred(1:2) - meas(1:2);
            error(3) = angle_normalization(pred(3), meas(3));

            # Compute Jacobian
            Jacobian = zeros(3, num_kin);
            first_k_laser_value = 1;
            for k = 1:num_kin
                last_k_laser_value = first_k_laser_value + dataset_size - 1; 

                # Prepare perturbed datasets
                laser_plus_i = laser_plus_all(first_k_laser_value:last_k_laser_value, :)(i, :);
                laser_minus_i = laser_minus_all(first_k_laser_value:last_k_laser_value, :)(i, :);

                Jacobian(1:2, k) = laser_plus_i(1:2) - laser_minus_i(1:2);
                Jacobian(3, k) = angle_normalization(laser_plus_i(3), laser_minus_i(3));

                first_k_laser_value += dataset_size;
            endfor

            # Scale Gradient
            Jacobian *= (0.5 / epsilon);

            # Apply Robust Estimator
            if (c > current_threshold)
                error *= sqrt(current_threshold / c);
                c = current_threshold;
            endif

            H += (Jacobian' * Jacobian);  #'
            b += (Jacobian' * error);     #'
            c += (error' * error);        #'
            
        endfor

        # Update Kinematic Parameters
        delta_x = -pinv(H) * b; 
        kinematic_parameters += delta_x;

        # Display Iteration Information
        display('Error:'), display(c);
        display('Iteration:'), display(iteration);
        display('Calibrated Kinematic Parameters:'), display(kinematic_parameters);

        # Plot Results
        calibrated_front_pose = final_robot_config(initial_state, max_enc_values, DATA, kinematic_parameters, steer_v);
        calibrated_pose_laser = calculate_laser_pose(kinematic_parameters, calibrated_front_pose);
        plot(calibrated_pose_laser(:,1), calibrated_pose_laser(:,2));
        axis([-5 4 -4 2]);
        title('Calibrated xy trajectory of the Robot ');    
        pause(2);
    endfor       
endfunction

# Calibrated 2D Laser Pose Trajectory + Calibrated Kinematic Parameters
epsilon = 1e-4;
dataset_size = 2434; 
n_iteration = 70;

kinematic_parameters = LeastSquares(kinematic_parameters, max_enc_values, initial_state, epsilon, n_iteration, dataset_size, num_kin, steer_v, DATA);

