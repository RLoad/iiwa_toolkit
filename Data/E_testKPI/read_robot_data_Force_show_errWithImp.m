clear; clc;close all;

% === Path to your file ===
% log_file = 'test error des with imp change/Force.txt';
% log_file = '1 change coverage rate/Force.txt';
% log_file = '2 change coverage rate to 60/Force.txt';
% log_file = '3 use right err and controller/3/Force.txt';
log_file = '4 use vel give a -z dir force disturb/15/Force.txt';

gif_name = 'tracking_animation_second_target_zeroed.gif';

fid = fopen(log_file, 'r');

times   = [];
tp_all  = [];
rp_all  = [];
tf_all  = [];
rf_all  = [];
errF_all = [];
err_all = [];
eigvals = [];
target_history = [];

% Store times for force data (to keep alignment)
times_force = [];

% Statistics for filtering
skipped_count = 0;
skipped_reasons = struct('non_finite', 0, 'too_large', 0);

radius=0.07

while ~feof(fid)
    line = strtrim(fgetl(fid));
    if startsWith(line,'time:')
        raw = extractAfter(line,'time:');
        t = str2double(raw);
        times(end+1) = t;
    elseif startsWith(line,'target_pose_:')
        vals = sscanf(line(length('target_pose_:')+1:end),'%f');
        target_pose = vals;
        target_history(:,end+1) = vals;
    elseif startsWith(line,'real_pose_:')
        vals = sscanf(line(length('real_pose_:')+1:end),'%f');
        real_pose = vals;

        rp_all(:,end+1) = real_pose;
        tp_all(:,end+1) = target_pose;
        err_all(end+1)  = norm(target_pose - real_pose)-radius;
    elseif startsWith(line,'real_force_filtered_:')
        vals = sscanf(line(length('real_force_filtered_:')+1:end),'%f');
        real_force_filtered = vals;
    elseif startsWith(line,'desired_force_:')
        vals = sscanf(line(length('desired_force_:')+1:end),'%f');
        desired_force = vals;

        % Validate force data - filter out unrealistic values
        % Check for NaN or Inf values first
        if any(~isfinite(desired_force)) || any(~isfinite(real_force_filtered))
            skipped_count = skipped_count + 1;
            skipped_reasons.non_finite = skipped_reasons.non_finite + 1;
            continue;  % Skip this data point
        end
        
        % Option 1: Calculate error only for Z-direction (most relevant for force tracking)
        % This avoids issues with corrupted X/Y components
        use_z_only = true;  % Set to false to use full 3D norm
        
        if use_z_only
            % Only use Z-direction (3rd component) for force error
            if length(desired_force) >= 3 && length(real_force_filtered) >= 3
                force_err_z = desired_force(3) - real_force_filtered(3);
                % Only add if Z-component is reasonable
                if abs(force_err_z) < 1e10 && isfinite(force_err_z)
                    if ~isempty(times)
                        rf_all(:,end+1) = real_force_filtered;
                        tf_all(:,end+1) = desired_force;
                        errF_all(end+1)  = abs(force_err_z);  % Use absolute value of Z-error
                        times_force(end+1) = times(end);
                    end
                else
                    skipped_count = skipped_count + 1;
                    skipped_reasons.too_large = skipped_reasons.too_large + 1;
                end
            end
        else
            % Option 2: Use full 3D norm but with validation
            % Check for extremely large values (likely data corruption or uninitialized values)
            max_reasonable_force = 1e10;  % Maximum reasonable force magnitude in N
            
            if any(abs(desired_force) > max_reasonable_force) || any(abs(real_force_filtered) > max_reasonable_force)
                skipped_count = skipped_count + 1;
                skipped_reasons.too_large = skipped_reasons.too_large + 1;
                if skipped_count <= 5  % Only show warning for first few occurrences
                    if ~isempty(times)
                        warning('Extremely large force values detected at time %.3f: desired=[%.2e %.2e %.2e], real=[%.2e %.2e %.2e]. Skipping.', ...
                            times(end), desired_force(1), desired_force(2), desired_force(3), ...
                            real_force_filtered(1), real_force_filtered(2), real_force_filtered(3));
                    end
                end
                continue;  % Skip this data point
            end

            % Only add force data if validation passes
            if ~isempty(times)
                rf_all(:,end+1) = real_force_filtered;
                tf_all(:,end+1) = desired_force;
                errF_all(end+1)  = norm(desired_force - real_force_filtered);
                times_force(end+1) = times(end);  % Store corresponding time
            end
        end
    elseif startsWith(line,'eig_value:')
        vals = sscanf(line(length('eig_value:')+1:end),'%f');
        eigvals(:,end+1) = vals;
    end
end
fclose(fid);

% Display filtering statistics
fprintf('\n===== Force Data Filtering Statistics =====\n');
fprintf('Total time points: %d\n', numel(times));
fprintf('Valid force data points: %d\n', numel(errF_all));
fprintf('Skipped data points: %d\n', skipped_count);
fprintf('  - Non-finite values: %d\n', skipped_reasons.non_finite);
fprintf('  - Too large values: %d\n', skipped_reasons.too_large);
fprintf('==========================================\n\n');

% Normalize time
if ~isempty(times)
    times = times - times(1);
end

%% === Detect second target pose segment ===
changes = find(any(abs(diff(target_history,1,2)) > 1e-6, 1));

if length(changes) < 1
    error('Only one target pose detected. No second segment found.');
elseif length(changes) == 1
    idx_start = changes(1)+1;
else
    idx_start = changes(2)+1;
end

% Slice data (second target pose)
times2   = times(idx_start:end);
tp2      = tp_all(:,idx_start:end);
rp2      = rp_all(:,idx_start:end);
err2     = errF_all(idx_start:end);
tf2 = tf_all(:,idx_start:end);
rf2      = rf_all(:,idx_start:end);
% Slice eigenvalues - ensure alignment with times2
if ~isempty(eigvals) && size(eigvals, 2) >= idx_start
    eig2 = eigvals(:, idx_start:end);
    % Ensure eig2 has same number of columns as times2
    if size(eig2, 2) ~= numel(times2)
        if size(eig2, 2) > numel(times2)
            eig2 = eig2(:, 1:numel(times2));
        else
            % Pad with last value if needed
            eig2 = [eig2, repmat(eig2(:, end), 1, numel(times2) - size(eig2, 2))];
        end
    end
else
    eig2 = [];
end

% For force data, need to find indices in times_force that correspond to times2
% Find force data points that fall within the second target segment
if ~isempty(times_force) && ~isempty(times2)
    % Normalize times for comparison
    times_force_norm = times_force - times(1);
    times2_norm = times2 - times(1);
    
    % Find indices in times_force that correspond to times2 range
    idx_force_start = find(times_force_norm >= times2_norm(1), 1, 'first');
    if isempty(idx_force_start)
        idx_force_start = 1;
    end
    
    % Extract force data for second target segment
    tf2 = tf_all(:, idx_force_start:end);
    rf2 = rf_all(:, idx_force_start:end);
    errF2 = errF_all(idx_force_start:end);
    times_force2 = times_force(idx_force_start:end);
    
    % Normalize times2 first (before interpolation)
    if ~isempty(times2)
        times2 = times2 - times2(1);
    end
    
    % Normalize times_force2 to match times2 (use times2 as reference)
    if ~isempty(times_force2) && ~isempty(times2)
        % Normalize times_force2 to start from 0
        times_force2 = times_force2 - times_force2(1);
        % Align time ranges: use the same time range as times2
        % Interpolate force error to times2 if needed
        if numel(times_force2) ~= numel(times2) || (numel(times_force2) > 1 && numel(times2) > 1 && max(abs(times_force2 - times2)) > 1e-3)
            % Interpolate errF2 to times2
            if numel(times_force2) > 1 && numel(times2) > 1
                errF2_interp = interp1(times_force2, errF2, times2, 'linear', 'extrap');
                errF2 = errF2_interp;
                fprintf('Interpolated force error data to match position time points (%d -> %d points)\n', ...
                    numel(times_force2), numel(times2));
            end
        end
        % Always use times2 as unified time for plotting
        times_force2 = times2;
    end
else
    tf2 = [];
    rf2 = [];
    errF2 = [];
    times_force2 = [];
    % Normalize times2 even if no force data
    if ~isempty(times2)
        times2 = times2 - times2(1);
    end
end

% Display statistics about force error
fprintf('\n===== Force Error Statistics =====\n');
fprintf('Total force error data points: %d\n', numel(errF2));
fprintf('Force error range: [%.6e, %.6e] N\n', min(errF2), max(errF2));
fprintf('Mean force error: %.6e N\n', mean(errF2));
fprintf('Median force error: %.6e N\n', median(errF2));
if any(errF2 > 1e6)
    fprintf('WARNING: Some force errors are very large (>1e6 N). This may indicate data issues.\n');
    fprintf('  Number of large errors: %d\n', nnz(errF2 > 1e6));
    fprintf('  Max error: %.6e N\n', max(errF2));
end
fprintf('=====================================\n\n');

%% === Shift coordinates so target pose becomes zero ===

t0 = tp2(:,1);  % reference target pose
tp2_shift = tp2 - t0;   % target trajectories translated
rp2_shift = rp2 - t0;   % real trajectories translated

% Error stays the same since computed from original difference
% (norm(target-real) invariant to translation)


mean_err = mean(err2);
disp(['Average tracking error (2nd target, shifted): ', num2str(mean_err), ' m']);

%% === Plot FINAL static figure first ===
figure('Position',[200 200 900 700]);

% Precompute reference circle
theta = linspace(0,2*pi,200);
circle_x = radius * cos(theta);
circle_y = radius * sin(theta);

% --- Subplot 1: 2D trajectory (final) ---
subplot(3,2,[1 3 5]);
plot(circle_x, circle_y, 'k--', 'LineWidth',1.5); hold on;
plot(tp2_shift(1,:), tp2_shift(2,:), 'kx', 'MarkerSize',8, 'LineWidth',2);
plot(rp2_shift(1,:), rp2_shift(2,:), 'b-', 'LineWidth',2);
plot(rp2_shift(1,end), rp2_shift(2,end), 'ro', 'MarkerSize',8, 'LineWidth',2);
hold off;
xlabel('X'); ylabel('Y');
title('Motion Trajectory');
legend('Desired traj','Attractor','Real traj','Last pose','Location','best');
axis equal; grid on;


% --- Subplot 2: Position X and Y over time ---
subplot(3,2,2);
plot(times2, rf2(1,:),'LineWidth',2); hold on;
% plot(times2, rf2(2,:),'LineWidth',2);
plot(times2, rf2(3,:),'LineWidth',2);
xlabel('Time (s)'); ylabel('Force (N)');
title('force X Z Over Time');
legend('X', 'Z','Location', 'best');
grid on;
hold off;

% --- Subplot 3: Force Error ---
subplot(3,2,4);
% plot(times2, err2,'r-','LineWidth',2);
% xlabel('Time (s)'); ylabel('Force Error (N)');
% title('Force Error Over Time');
grid on;
% Use unified time (times2) for all plots
if ~isempty(errF2) && numel(errF2) == numel(times2)
    plot(times2, errF2,'r-','LineWidth',2);
    xlabel('Time (s)'); ylabel('Force Error (N)');
    title('Force Error (Z-direction) Over Time');
    grid on;
else
    text(0.5, 0.5, sprintf('No valid force error data\n(Expected %d points, got %d)', ...
         numel(times2), numel(errF2)), ...
         'HorizontalAlignment', 'center', 'Units', 'normalized', ...
         'FontSize', 12);
    xlabel('Time (s)'); ylabel('Force Error (N)');
    title('Force Error (Z-direction) Over Time');
    grid on;
    if ~isempty(errF2)
        warning('Force error data size mismatch: times2 has %d points, errF2 has %d points', ...
            numel(times2), numel(errF2));
    end
end

% --- Subplot 4: Eigenvalues ---
subplot(3,2,6);
% Ensure eig2 has same number of columns as times2
if ~isempty(eig2) && size(eig2, 2) == numel(times2)
    plot(times2, eig2','LineWidth',2);
    xlabel('Time (s)'); ylabel('Eigenvalue (motion direction)');
    title('Impedance Parameters Over Time');
    grid on;
    if size(eig2, 1) > 1
        legend(arrayfun(@(i) sprintf('Eig %d', i), 1:size(eig2,1), 'UniformOutput', false), ...
               'Location', 'best');
    end
else
    text(0.5, 0.5, sprintf('Eigenvalue data size mismatch\n(times2: %d, eig2: %d)', ...
         numel(times2), size(eig2, 2)), ...
         'HorizontalAlignment', 'center', 'Units', 'normalized', ...
         'FontSize', 12);
    xlabel('Time (s)'); ylabel('Eigenvalues');
    title('Impedance Parameters (Eigenvalues) Over Time');
    grid on;
end



% Add title for all subplots (compatible with MATLAB 2017)
% ha = axes('Position', [0 0 1 1], 'Visible', 'off');
% text(0.5, 0.98, 'Final Results (Second Target Only)', ...
%      'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
%      'FontSize', 14, 'FontWeight', 'bold', 'Parent', ha);



%% === Compute average speed (second target) ===
% Use shifted or unshifted poses; translation does not change distances
% Choose 2D (x,y) or 3D (x,y,z) automatically
dim = min(3, size(rp2_shift,1));   % if only x,y exist -> dim=2; if x,y,z exist -> dim=3
pos = rp2_shift(1:dim, :);

dpos = diff(pos, 1, 2);           % position increments
ds   = sqrt(sum(dpos.^2, 1));       % distance per step
dt   = diff(times2);              % time per step

% Guard against invalid time steps (e.g., duplicated timestamps)
valid = isfinite(ds) & isfinite(dt) & (dt > 0);

total_dist = sum(ds(valid));
total_time = sum(dt(valid));

avg_speed = total_dist / total_time;            % time-weighted mean speed (recommended)
mean_step_speed = mean(ds(valid) ./ dt(valid)); % simple mean of instantaneous speeds (for reference)

disp(['Total distance (2nd target): ', num2str(total_dist), ' m']);
disp(['Total time     (2nd target): ', num2str(total_time), ' s']);
disp(['Average speed  (distance/time): ', num2str(avg_speed), ' m/s']);
disp(['Mean step speed (mean(ds/dt)): ', num2str(mean_step_speed), ' m/s']);

%% === Ask user if we should animate ===
% answer = input('Generate GIF animation? (y/n): ','s');
% 
% if ~(strcmpi(answer,'y') || strcmpi(answer,'yes'))
%     disp('Animation skipped. Script finished.');
%     return;
% end
% 
% %% === Animation plotting ===
% disp('Generating animation GIF...');
% figure('Position',[200 200 900 700]);
% num_frames = length(times2);
% 
% for k = 1:num_frames
% 
%     % ----- Subplot 1: 2D trajectory (XY) -----
%     subplot(3,1,1);
%     plot(circle_x, circle_y, 'k--', 'LineWidth', 1.5); hold on;
%     plot(tp2_shift(1,:), tp2_shift(2,:), 'kx', 'MarkerSize',8, 'LineWidth',2);
%     plot(rp2_shift(1,1:k), rp2_shift(2,1:k), 'b-', 'LineWidth',2);
%     plot(rp2_shift(1,k), rp2_shift(2,k), 'ro', 'MarkerSize',8, 'LineWidth',2);
%     hold off;
%     xlabel('X (shifted)'); ylabel('Y (shifted)');
%     title(sprintf('Shifted Trajectory (Second Target) frame %d/%d',k,num_frames));
%     legend('Desired circle','Target (0)','Real traj','Current real','Location','best');
%     axis equal; grid on;
% 
%     % ----- Subplot 2: Error -----
%     subplot(3,1,2);
%     plot(times2(1:k), err2(1:k),'r-','LineWidth',2);
%     xlabel('Time (s)'); ylabel('Error (m)');
%     title('Error Norm Evolution');
%     grid on; xlim([times2(1) times2(end)]);
% 
%     % ----- Subplot 3: Eigenvalues -----
%     subplot(3,1,3);
%     plot(times2(1:k), eig2(:,1:k)','LineWidth',2);
%     xlabel('Time (s)'); ylabel('Eigenvalues');
%     title('Eigenvalue Evolution');
%     grid on; xlim([times2(1) times2(end)]);
% 
%     drawnow;
% 
%     % ---- Save frame to GIF ----
%     frame = getframe(gcf);
%     [imind, cm] = rgb2ind(frame2im(frame),256);
%     if k == 1
%         imwrite(imind,cm,gif_name,'gif','Loopcount',inf,'DelayTime',0.05);
%     else
%         imwrite(imind,cm,gif_name,'gif','WriteMode','append','DelayTime',0.05);
%     end
% end
% 
% disp(['GIF saved as: ', gif_name]);
