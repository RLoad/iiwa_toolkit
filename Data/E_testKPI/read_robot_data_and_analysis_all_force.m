clear; clc; close all;

%% ===== User config =====
base_dir = '4 use vel give a -z dir force disturb';   % 你的主目录
folder_ids = 1:15;                               % 子文件夹编号 1~12
radius = 0.07;                                  % 你的半径（用于速度计算，不再用于误差）
use_second_target_only = true;

% If true: also plot instantaneous speed vs instantaneous error (all runs pooled)
plot_instant_scatter = true;

%% ===== Batch processing =====
avg_speed_all = nan(size(folder_ids));
mean_err_all  = nan(size(folder_ids));
mean_desired_force_mag_all = nan(size(folder_ids));  % mean desired force magnitude per run
total_time_all = nan(size(folder_ids));
total_dist_all = nan(size(folder_ids));

v_inst_all = [];   % pooled instantaneous speeds
e_inst_all = [];   % pooled instantaneous errors
run_id_all = [];   % which folder each point comes from (for optional coloring/analysis)

% Store force data for plotting
all_times_force = cell(numel(folder_ids), 1);
all_desired_force_z = cell(numel(folder_ids), 1);
all_real_force_z = cell(numel(folder_ids), 1);

for ii = 1:numel(folder_ids)
    k = folder_ids(ii);

    log_file = fullfile(base_dir, num2str(k), 'Force.txt');
    if ~isfile(log_file)
        warning('File not found: %s (skipped)', log_file);
        continue;
    end

    % --- Parse and compute metrics ---
    out = parse_force_file_compute_metrics(log_file, radius, use_second_target_only);

    avg_speed_all(ii)  = out.avg_speed;
    mean_err_all(ii)   = out.mean_err;
    mean_desired_force_mag_all(ii) = out.mean_desired_force_mag;
    total_time_all(ii) = out.total_time;
    total_dist_all(ii) = out.total_dist;

    % Store force data for plotting
    all_times_force{ii} = out.times_force;
    all_desired_force_z{ii} = out.desired_force_z;
    all_real_force_z{ii} = out.real_force_z;

    if plot_instant_scatter && ~isempty(out.v_inst)
        v_inst_all = [v_inst_all, out.v_inst];
        e_inst_all = [e_inst_all, out.e_inst];
        run_id_all = [run_id_all, k * ones(1, numel(out.v_inst))];
    end

    fprintf('[%d] mean_force_err=%.6f N, avg_speed=%.6f m/s, time=%.3f s, dist=%.3f m\n', ...
        k, out.mean_err, out.avg_speed, out.total_time, out.total_dist);
end

%% ===== Plot: 3D - average speed vs mean error vs desired force magnitude (per run) =====
valid_run = isfinite(avg_speed_all) & isfinite(mean_err_all) & isfinite(mean_desired_force_mag_all);

figure('Position',[200 200 1000 800]);

scatter3(avg_speed_all(valid_run), mean_err_all(valid_run), mean_desired_force_mag_all(valid_run), 80, 'filled', 'MarkerFaceAlpha', 0.7); 
grid on; hold on;
xlabel('Average speed (m/s)');
ylabel('Mean force error (N)');
zlabel('Desired force magnitude (N)');
title('3D: Average Speed vs Mean Force Error vs Desired Force Magnitude (2nd target)');

% Add labels 1~12 on points
xs = avg_speed_all(valid_run);
ys = mean_err_all(valid_run);
zs = mean_desired_force_mag_all(valid_run);
ks = folder_ids(valid_run);
for i = 1:numel(xs)
    text(xs(i), ys(i), zs(i), sprintf('  %d', ks(i)), 'FontSize', 10, 'VerticalAlignment', 'middle');
end

view(45, 30); % Set viewing angle for better visualization
hold off;

%% ===== Plot: instantaneous speed vs instantaneous error (all runs pooled) =====
if plot_instant_scatter && ~isempty(v_inst_all)
    figure('Position',[300 300 900 600]);
    scatter(v_inst_all, e_inst_all, 8, 'filled'); grid on;
    xlabel('Instantaneous speed (m/s)');
    ylabel('Instantaneous force error (N)');
    title('Instantaneous Speed vs Force Error (all time steps pooled, 2nd target)');
end

%% ===== Plot: 5x5 subplots - Desired vs Real Force (Z-direction) over time =====
figure('Position',[50 50 2000 2000]);
num_subplots = 25;  % 5x5 = 25 subplots
valid_experiments = [];

% Collect valid experiments
for ii = 1:numel(folder_ids)
    if ~isempty(all_times_force{ii}) && ~isempty(all_desired_force_z{ii}) && ~isempty(all_real_force_z{ii})
        valid_experiments(end+1) = ii;
    end
end

% Plot up to 25 experiments
num_to_plot = min(num_subplots, numel(valid_experiments));
for plot_idx = 1:num_to_plot
    exp_idx = valid_experiments(plot_idx);
    k = folder_ids(exp_idx);
    
    subplot(4, 4, plot_idx);
    times_plot = all_times_force{exp_idx};
    desired_z = all_desired_force_z{exp_idx};
    real_z = all_real_force_z{exp_idx};
    
    plot(times_plot, desired_z, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Desired'); hold on;
    plot(times_plot, real_z, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Real'); 
    grid on;
    xlabel('Time (s)');
    ylabel('Force Z (N)');
    title(sprintf('Exp %d', k));
    legend('Location', 'best', 'FontSize', 7);
    hold off;
end

% Hide unused subplots
for plot_idx = (num_to_plot + 1):num_subplots
    subplot(4, 4, plot_idx);
    axis off;
end

sgtitle('Desired Force (Z) vs Real Force (Z) Over Time - All Experiments', 'FontSize', 14, 'FontWeight', 'bold');

%% ===== Local function =====
function out = parse_force_file_compute_metrics(log_file, radius, second_target_only)

    fid = fopen(log_file, 'r');
    if fid < 0
        error('Cannot open file: %s', log_file);
    end

    times   = [];
    tp_all  = [];
    rp_all  = [];
    err_all = [];  % force error (z-direction)
    target_history = [];
    
    % Store force data for plotting
    desired_force_z_all = [];
    real_force_z_all = [];
    times_force_all = [];
    
    % Store full desired_force vectors for magnitude calculation
    desired_force_all = [];  % Will store as 3xN matrix

    target_pose = [];
    real_force_filtered = [];
    desired_force = [];
    current_time = [];

    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if startsWith(line,'time:')
            raw = extractAfter(line,'time:');
            current_time = str2double(raw);
            times(end+1) = current_time;

        elseif startsWith(line,'real_force_filtered_:')
            vals = sscanf(line(length('real_force_filtered_:')+1:end),'%f');
            real_force_filtered = vals;

        elseif startsWith(line,'desired_force_:')
            vals = sscanf(line(length('desired_force_:')+1:end),'%f');
            desired_force = vals;

        elseif startsWith(line,'target_pose_:')
            vals = sscanf(line(length('target_pose_:')+1:end),'%f');
            target_pose = vals;
            target_history(:,end+1) = vals;

        elseif startsWith(line,'real_pose_:')
            vals = sscanf(line(length('real_pose_:')+1:end),'%f');
            real_pose = vals;

            % store position data (needed for speed calculation)
            rp_all(:,end+1) = real_pose;
            tp_all(:,end+1) = target_pose;
            
            % Calculate z-direction force error and store force data
            if ~isempty(real_force_filtered) && ~isempty(desired_force)
                % Assuming z is the 3rd component (index 3)
                if length(real_force_filtered) >= 3 && length(desired_force) >= 3
                    force_err_z = desired_force(3) - real_force_filtered(3);
                    err_all(end+1) = force_err_z;
                    
                    % Store force data for plotting (use last time value)
                    if ~isempty(times)
                        desired_force_z_all(end+1) = desired_force(3);
                        real_force_z_all(end+1) = real_force_filtered(3);
                        times_force_all(end+1) = times(end);
                        
                        % Store full desired_force vector
                        if isempty(desired_force_all)
                            desired_force_all = desired_force(:);
                        else
                            desired_force_all(:, end+1) = desired_force(:);
                        end
                    else
                        err_all(end) = NaN;
                    end
                else
                    err_all(end+1) = NaN;
                    if ~isempty(times)
                        desired_force_z_all(end+1) = NaN;
                        real_force_z_all(end+1) = NaN;
                        times_force_all(end+1) = times(end);
                        % Store NaN vector for desired_force
                        if isempty(desired_force_all)
                            desired_force_all = [NaN; NaN; NaN];
                        else
                            desired_force_all(:, end+1) = [NaN; NaN; NaN];
                        end
                    end
                end
            else
                err_all(end+1) = NaN;
                if ~isempty(times)
                    desired_force_z_all(end+1) = NaN;
                    real_force_z_all(end+1) = NaN;
                    times_force_all(end+1) = times(end);
                    % Store NaN vector for desired_force
                    if isempty(desired_force_all)
                        desired_force_all = [NaN; NaN; NaN];
                    else
                        desired_force_all(:, end+1) = [NaN; NaN; NaN];
                    end
                end
            end

        elseif startsWith(line,'----------------------------------------')
            % Reset force variables at separator (for next data block)
            real_force_filtered = [];
            desired_force = [];
            target_pose = [];
        end
    end
    fclose(fid);

    % Normalize time
    times = times - times(1);

    % Decide segment
    if second_target_only
        changes = find(any(abs(diff(target_history,1,2)) > 1e-6, 1));
        if isempty(changes)
            error('Only one target pose detected in %s; cannot extract second segment.', log_file);
        elseif numel(changes) == 1
            idx_start = changes(1) + 1;
        else
            idx_start = changes(2) + 1;
        end
    else
        idx_start = 1;
    end

    % Slice
    times2 = times(idx_start:end);
    tp2    = tp_all(:,idx_start:end);
    rp2    = rp_all(:,idx_start:end);
    err2   = err_all(idx_start:end);
    
    % Slice force data
    times_force2 = times_force_all(idx_start:end);
    desired_force_z2 = desired_force_z_all(idx_start:end);
    real_force_z2 = real_force_z_all(idx_start:end);
    
    % Slice desired_force vectors
    if ~isempty(desired_force_all)
        desired_force_all2 = desired_force_all(:, idx_start:end);
        % Calculate mean desired_force vector (excluding NaN values)
        valid_cols = all(isfinite(desired_force_all2), 1);
        if any(valid_cols)
            mean_desired_force = mean(desired_force_all2(:, valid_cols), 2);
            out.mean_desired_force_mag = norm(mean_desired_force);
        else
            out.mean_desired_force_mag = NaN;
        end
    else
        out.mean_desired_force_mag = NaN;
    end
    
    % Normalize force times
    if ~isempty(times_force2)
        times_force2 = times_force2 - times_force2(1);
    end

    % Shift by first target pose in this segment (translation-invariant for speed)
    t0 = tp2(:,1);
    rp2_shift = rp2 - t0;

    % Compute mean error (force error)
    out.mean_err = mean(err2(~isnan(err2)));

    % Compute avg speed (distance/time)
    dim = min(3, size(rp2_shift,1));
    pos = rp2_shift(1:dim, :);

    dpos = diff(pos, 1, 2);
    ds   = vecnorm(dpos, 2, 1);
    dt   = diff(times2);

    valid = isfinite(ds) & isfinite(dt) & (dt > 0);

    out.total_dist = sum(ds(valid));
    out.total_time = sum(dt(valid));
    out.avg_speed  = out.total_dist / out.total_time;

    % Instantaneous speed vs instantaneous error (aligned to step i: between i and i+1)
    if any(valid)
        v_inst = ds(valid) ./ dt(valid);
        e_step = err2(2:end);     % step-wise error at the end of each interval
        e_inst = e_step(valid);   % apply same mask as ds/dt
        out.v_inst = v_inst;
        out.e_inst = e_inst;
    else
        out.v_inst = [];
        out.e_inst = [];
    end
    
    % Store force data for plotting
    out.times_force = times_force2;
    out.desired_force_z = desired_force_z2;
    out.real_force_z = real_force_z2;
end
