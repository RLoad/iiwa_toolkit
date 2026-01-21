clear; clc; close all;

%% ===== User config =====
base_dir = '3 use right err and controller';   % ???????????????
folder_ids = 1:12;                               % ?????????????????? 1~9
radius = 0.07;                                  % ????????????????????????????????????
use_second_target_only = true;

% If true: also plot instantaneous speed vs instantaneous error (all runs pooled)
plot_instant_scatter = true;

%% ===== Interactive data segmentation: Click 4 points for each experiment =====
% This section allows user to click 4 time points on each experiment's position error plot
% to segment the data into 3 segments: segment 1 (t1-t2), segment 2 (t2-t3), segment 3 (t3-t4)
click_times = cell(size(folder_ids));  % Store 4 time points for each experiment

% Check if saved click times file exists
saved_click_file = fullfile(base_dir, 'saved_click_times.mat');
use_saved_data = false;

if exist(saved_click_file, 'file') == 2
    fprintf('\n===== Found saved click times file =====\n');
    load(saved_click_file, 'saved_click_times', 'saved_folder_ids');
    
    % Check if saved data matches current folder_ids
    if isequal(saved_folder_ids, folder_ids) && numel(saved_click_times) == numel(folder_ids)
        fprintf('Saved data matches current experiments. Using saved click times.\n');
        click_times = saved_click_times;
        use_saved_data = true;
    else
        fprintf('Saved data does not match current experiments. Will re-click.\n');
        fprintf('  Saved folder_ids: %s\n', mat2str(saved_folder_ids));
        fprintf('  Current folder_ids: %s\n', mat2str(folder_ids));
    end
end

if ~use_saved_data
    fprintf('\n===== Interactive Segmentation: Click 4 points for each experiment =====\n');
    fprintf('For each experiment, click 4 points in order: start, seg1_end, seg2_end, end\n');
    fprintf('Press any key to continue to next experiment after clicking 4 points.\n\n');

    for ii = 1:numel(folder_ids)
    k = folder_ids(ii);
    
    log_file = fullfile(base_dir, num2str(k), 'Force.txt');
    if exist(log_file, 'file') ~= 2
        warning('File not found: %s (skipped)', log_file);
        click_times{ii} = [];  % Empty for skipped experiments
        continue;
    end
    
    % Parse data to get position error for plotting
    out_temp = parse_force_file_compute_metrics(log_file, radius, use_second_target_only);
    
    if isempty(out_temp.times) || isempty(out_temp.errors)
        warning('No valid data in experiment %d (skipped)', k);
        click_times{ii} = [];
        continue;
    end
    
    % Plot position error over time
    figure('Position', [100 100 1200 600]);
    plot(out_temp.times, out_temp.errors * 100, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Position Error (cm)');
    title(sprintf('Experiment %d: Click 4 points (start, seg1_end, seg2_end, end)', k), ...
          'FontSize', 12, 'FontWeight', 'bold');
    
    % Get 4 clicks from user
    fprintf('Experiment %d: Click 4 points on the plot...\n', k);
    [x_click, ~] = ginput(4);
    
    % Sort to ensure time order (t1 < t2 < t3 < t4)
    x_click = sort(x_click);
    
    % Validate that clicks are within data range
    t_min = min(out_temp.times);
    t_max = max(out_temp.times);
    if x_click(1) < t_min || x_click(4) > t_max
        warning('Some clicks are outside data range for experiment %d. Clipping to valid range.', k);
        x_click(1) = max(x_click(1), t_min);
        x_click(4) = min(x_click(4), t_max);
    end
    
    % Store the 4 time points
    click_times{ii} = x_click;
    
    fprintf('  Stored time points: %.3f, %.3f, %.3f, %.3f s\n', x_click(1), x_click(2), x_click(3), x_click(4));
    
    close(gcf);  % Close the figure after clicking
    end

    % Save click times to file
    fprintf('\n===== Saving click times to file =====\n');
    saved_click_times = click_times;
    saved_folder_ids = folder_ids;
    save(saved_click_file, 'saved_click_times', 'saved_folder_ids');
    fprintf('Click times saved to: %s\n', saved_click_file);
    fprintf('===== Interactive segmentation completed =====\n\n');
else
    fprintf('===== Using saved click times (skipping interactive clicking) =====\n\n');
end

%% ===== Batch processing =====
avg_speed_all = nan(size(folder_ids));
mean_err_all  = nan(size(folder_ids));
std_err_all   = nan(size(folder_ids));  % Standard deviation of tracking error
total_time_all = nan(size(folder_ids));
total_dist_all = nan(size(folder_ids));

% New arrays for segmented data analysis
avg_speed_segments_1_3 = nan(size(folder_ids));
mean_err_segments_1_3 = nan(size(folder_ids));
std_err_segments_1_3 = nan(size(folder_ids));  % Standard deviation for segments 1+3
setting_time_segment_2 = nan(size(folder_ids));

v_inst_all = [];   % pooled instantaneous speeds
e_inst_all = [];   % pooled instantaneous errors
run_id_all = [];   % which folder each point comes from (for optional coloring/analysis)

% Arrays to store time and position error for plotting
all_times_error = cell(size(folder_ids));
all_position_error = cell(size(folder_ids));

for ii = 1:numel(folder_ids)
    k = folder_ids(ii);

    log_file = fullfile(base_dir, num2str(k), 'Force.txt');
    if exist(log_file, 'file') ~= 2
        warning('File not found: %s (skipped)', log_file);
        continue;
    end

    % Get time segments for this experiment (if available)
    time_seg = [];
    if ~isempty(click_times{ii}) && numel(click_times{ii}) == 4
        time_seg = click_times{ii};
    end

    % --- Parse and compute metrics with time segmentation ---
    out = parse_force_file_compute_metrics(log_file, radius, use_second_target_only, time_seg);

    % Store original metrics (for backward compatibility)
    avg_speed_all(ii)  = out.avg_speed;
    mean_err_all(ii)   = out.mean_err;
    std_err_all(ii)    = out.std_err;
    total_time_all(ii) = out.total_time;
    total_dist_all(ii) = out.total_dist;

    % Store segmented metrics
    if ~isempty(time_seg)
        % Use segments 1+3 for speed calculation
        avg_speed_segments_1_3(ii) = out.avg_speed;  % Already calculated from segments 1+3
        mean_err_segments_1_3(ii) = out.mean_err_segments_1_3;
        std_err_segments_1_3(ii) = out.std_err_segments_1_3;
        setting_time_segment_2(ii) = out.setting_time;
    else
        % If no segmentation, use original values
        avg_speed_segments_1_3(ii) = out.avg_speed;
        mean_err_segments_1_3(ii) = out.mean_err;
        std_err_segments_1_3(ii) = out.std_err;
        setting_time_segment_2(ii) = nan;
    end

    % Store time and position error arrays for plotting
    all_times_error{ii} = out.times;
    all_position_error{ii} = out.errors;

    if plot_instant_scatter && ~isempty(out.v_inst)
        v_inst_all = [v_inst_all, out.v_inst];
        e_inst_all = [e_inst_all, out.e_inst];
        run_id_all = [run_id_all, k * ones(1, numel(out.v_inst))];
    end

    fprintf('[%d] mean_err=%.6f m, std_err=%.6f m, avg_speed=%.6f m/s, time=%.3f s, dist=%.3f m', ...
        k, out.mean_err, out.std_err, out.avg_speed, out.total_time, out.total_dist);
    if ~isempty(time_seg)
        fprintf(', seg1+3_err=%.6f m (std=%.6f m), setting_time=%.3f s', ...
            mean_err_segments_1_3(ii), std_err_segments_1_3(ii), setting_time_segment_2(ii));
    end
    fprintf('\n');
end

%% ===== Plot: average speed vs mean error (per run) - Using Segments 1+3 =====
% Use segmented data: segments 1+3 combined for error calculation
valid_run = isfinite(avg_speed_segments_1_3) & isfinite(mean_err_segments_1_3) & isfinite(std_err_segments_1_3);

figure('Position',[200 200 900 700]);

xs = avg_speed_segments_1_3(valid_run);
ys_mean = mean_err_segments_1_3(valid_run)*100;
ys_std = std_err_segments_1_3(valid_run)*100;

% Plot with error bars showing mean ?? std
errorbar(xs, ys_mean, ys_std, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b', ...
         'LineWidth', 1.5, 'CapSize', 8); 
grid on; hold on;
xlabel('Average speed (m/s)');
ylabel('Mean tracking error (cm)');
title('Position tracking error in 12 trails without consider disturbance phase (Mean + Std)');

% Add labels on points
% ks = folder_ids(valid_run);
% for i = 1:numel(xs)
%     text(xs(i), ys_mean(i), sprintf('  %d', ks(i)), 'FontSize', 10, 'VerticalAlignment', 'middle');
% end

hold off;

%% ===== Plot: average speed vs setting time (per run) - Using Segment 2 =====
% Use segmented data: segments 1+3 for speed, segment 2 for setting time
valid_run_st = isfinite(avg_speed_segments_1_3) & isfinite(setting_time_segment_2);

figure('Position',[200 200 900 700]);

scatter(avg_speed_segments_1_3(valid_run_st), setting_time_segment_2(valid_run_st), 80, 'filled'); grid on; hold on;
xlabel('Average speed (m/s)');
ylabel('Setting Time (s)');
title('Relationship: Average Speed vs Setting Time (Segment 2)');

% Add labels on points
xs_st = avg_speed_segments_1_3(valid_run_st);
ys_st = setting_time_segment_2(valid_run_st);
ks_st = folder_ids(valid_run_st);
for i = 1:numel(xs_st)
    text(xs_st(i), ys_st(i), sprintf('  %d', ks_st(i)), 'FontSize', 10, 'VerticalAlignment', 'middle');
end
hold off;

%% ===== Plot: mean error vs std error (per run) - Using Segments 1+3 =====
% Show relationship between mean and standard deviation of tracking error
valid_run_std = isfinite(mean_err_segments_1_3) & isfinite(std_err_segments_1_3);

figure('Position',[200 200 900 700]);

scatter(mean_err_segments_1_3(valid_run_std)*100, std_err_segments_1_3(valid_run_std)*100, 80, 'filled'); 
grid on; hold on;
xlabel('Mean tracking error (cm)');
ylabel('Standard deviation of tracking error (cm)');
title('Relationship: Mean vs Standard Deviation of Tracking Error (Segments 1+3)');

% Add labels on points
xs_std = mean_err_segments_1_3(valid_run_std)*100;
ys_std = std_err_segments_1_3(valid_run_std)*100;
ks_std = folder_ids(valid_run_std);
for i = 1:numel(xs_std)
    text(xs_std(i), ys_std(i), sprintf('  %d', ks_std(i)), 'FontSize', 10, 'VerticalAlignment', 'middle');
end
hold off;

%% ===== Plot: instantaneous speed vs instantaneous error (all runs pooled) =====
% subplot(2,1,2);
% if plot_instant_scatter && ~isempty(v_inst_all)
%     scatter(v_inst_all, e_inst_all, 8, 'filled'); grid on;
%     xlabel('Instantaneous speed (m/s)');
%     ylabel('Instantaneous tracking error (m)');
%     title('Instantaneous Speed vs Error (all time steps pooled, 2nd target)');
% else
%     axis off;
%     text(0.1, 0.5, 'Instantaneous scatter disabled or no valid data.', 'FontSize', 12);
% end
% 
% sgtitle(sprintf('Batch results from %s/{1..9}/Force.txt', base_dir));


%% ===== Plot: 4x4 subplots - Position Error over time =====
figure('Position',[50 50 2000 2000]);
num_subplots = 16;  % 4x4 = 16 subplots
valid_experiments = [];

% Collect valid experiments
for ii = 1:numel(folder_ids)
    if ~isempty(all_times_error{ii}) && ~isempty(all_position_error{ii})
        valid_experiments(end+1) = ii;
    end
end

% Plot up to 16 experiments
num_to_plot = min(num_subplots, numel(valid_experiments));
for plot_idx = 1:num_to_plot
    exp_idx = valid_experiments(plot_idx);
    k = folder_ids(exp_idx);
    
    subplot(4, 4, plot_idx);
    times_plot = all_times_error{exp_idx};
    position_error = all_position_error{exp_idx};
    
    plot(times_plot, position_error * 100, 'r-', 'LineWidth', 1.5); 
    grid on;
    xlabel('Time (s)');
    ylabel('Position Error (cm)');
    title(sprintf('Exp %d', k));
    hold off;
end

% Hide unused subplots
for plot_idx = (num_to_plot + 1):num_subplots
    subplot(4, 4, plot_idx);
    axis off;
end

% Add title for all subplots (compatible with MATLAB 2017)
% Create an invisible axes covering the whole figure
ha = axes('Position', [0 0 1 1], 'Visible', 'off');
text(0.5, 0.98, 'Position Error Over Time - All Experiments', ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
     'FontSize', 14, 'FontWeight', 'bold', 'Parent', ha);



%% ===== Local function =====
function out = parse_force_file_compute_metrics(log_file, radius, second_target_only, time_segments)
    % time_segments: optional array of 4 time points [t1, t2, t3, t4] for data segmentation
    % If provided, data will be segmented into 3 parts: seg1 (t1-t2), seg2 (t2-t3), seg3 (t3-t4)
    if nargin < 4
        time_segments = [];
    end

    fid = fopen(log_file, 'r');
    if fid < 0
        error('Cannot open file: %s', log_file);
    end

    times   = [];
    tp_all  = [];
    rp_all  = [];
    err_all = [];
    target_history = [];

    target_pose = [];

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

            % store
            rp_all(:,end+1) = real_pose;
            tp_all(:,end+1) = target_pose;
            err_all(end+1)  = norm(target_pose - real_pose) - radius;
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

    % Initialize output struct
    out = struct();
    segmentation_applied = false;

    % Apply time segmentation if provided
    if ~isempty(time_segments) && numel(time_segments) == 4
        t1 = time_segments(1);
        t2 = time_segments(2);
        t3 = time_segments(3);
        t4 = time_segments(4);
        
        % Find indices corresponding to the time points
        idx_t1 = find(times2 >= t1, 1, 'first');
        idx_t2 = find(times2 >= t2, 1, 'first');
        idx_t3 = find(times2 >= t3, 1, 'first');
        idx_t4 = find(times2 >= t4, 1, 'first');
        
        if isempty(idx_t1) || isempty(idx_t4)
            warning('Time segmentation points out of range, using all data');
        else
            % Clip to valid range
            idx_t1 = max(1, idx_t1);
            idx_t2 = max(idx_t1, min(idx_t2, numel(times2)));
            idx_t3 = max(idx_t2, min(idx_t3, numel(times2)));
            idx_t4 = min(numel(times2), idx_t4);
            
            % Extract segments
            % Segment 1: t1 to t2
            seg1_times = times2(idx_t1:idx_t2);
            seg1_tp = tp2(:, idx_t1:idx_t2);
            seg1_rp = rp2(:, idx_t1:idx_t2);
            seg1_err = err2(idx_t1:idx_t2);
            
            % Segment 2: t2 to t3
            seg2_times = times2(idx_t2:idx_t3);
            seg2_tp = tp2(:, idx_t2:idx_t3);
            seg2_rp = rp2(:, idx_t2:idx_t3);
            seg2_err = err2(idx_t2:idx_t3);
            
            % Segment 3: t3 to t4
            seg3_times = times2(idx_t3:idx_t4);
            seg3_tp = tp2(:, idx_t3:idx_t4);
            seg3_rp = rp2(:, idx_t3:idx_t4);
            seg3_err = err2(idx_t3:idx_t4);
            
            % Store segment information in output
            out.seg1_times = seg1_times;
            out.seg1_tp = seg1_tp;
            out.seg1_rp = seg1_rp;
            out.seg1_err = seg1_err;
            
            out.seg2_times = seg2_times;
            out.seg2_tp = seg2_tp;
            out.seg2_rp = seg2_rp;
            out.seg2_err = seg2_err;
            
            out.seg3_times = seg3_times;
            out.seg3_tp = seg3_tp;
            out.seg3_rp = seg3_rp;
            out.seg3_err = seg3_err;
            
            % For combined segments 1+3 (used for error calculation)
            seg1_3_times = [seg1_times, seg3_times];
            seg1_3_tp = [seg1_tp, seg3_tp];
            seg1_3_rp = [seg1_rp, seg3_rp];
            seg1_3_err = [seg1_err, seg3_err];
            
            % Use segments 1+3 for main calculations
            times2 = seg1_3_times;
            tp2 = seg1_3_tp;
            rp2 = seg1_3_rp;
            err2 = seg1_3_err;
            
            % Calculate speed separately for segments 1 and 3 to avoid position discontinuity
            % Segment 1 speed calculation
            t0_seg1 = seg1_tp(:,1);
            rp_seg1_shift = seg1_rp - t0_seg1;
            dim = min(3, size(rp_seg1_shift,1));
            pos_seg1 = rp_seg1_shift(1:dim, :);
            dpos_seg1 = diff(pos_seg1, 1, 2);
            ds_seg1 = sqrt(sum(dpos_seg1.^2, 1));
            dt_seg1 = diff(seg1_times);
            valid_seg1 = isfinite(ds_seg1) & isfinite(dt_seg1) & (dt_seg1 > 0);
            dist_seg1 = sum(ds_seg1(valid_seg1));
            time_seg1 = sum(dt_seg1(valid_seg1));
            
            % Segment 3 speed calculation
            t0_seg3 = seg3_tp(:,1);
            rp_seg3_shift = seg3_rp - t0_seg3;
            pos_seg3 = rp_seg3_shift(1:dim, :);
            dpos_seg3 = diff(pos_seg3, 1, 2);
            ds_seg3 = sqrt(sum(dpos_seg3.^2, 1));
            dt_seg3 = diff(seg3_times);
            valid_seg3 = isfinite(ds_seg3) & isfinite(dt_seg3) & (dt_seg3 > 0);
            dist_seg3 = sum(ds_seg3(valid_seg3));
            time_seg3 = sum(dt_seg3(valid_seg3));
            
            % Combined speed for segments 1+3
            out.total_dist = dist_seg1 + dist_seg3;
            out.total_time = time_seg1 + time_seg3;
            out.avg_speed = out.total_dist / out.total_time;
            segmentation_applied = true;
        end
    end

    % If no segmentation, use original calculation
    if ~segmentation_applied
        % Shift by first target pose in this segment (translation-invariant for speed)
        t0 = tp2(:,1);
        rp2_shift = rp2 - t0;

        % Compute mean error and standard deviation
        out.mean_err = mean(err2);
        out.std_err = std(err2);

        % Compute avg speed (distance/time)
        dim = min(3, size(rp2_shift,1));
        pos = rp2_shift(1:dim, :);

        dpos = diff(pos, 1, 2);
        ds   = sqrt(sum(dpos.^2, 1));
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
    else
        % Mean error and standard deviation for segments 1+3
        out.mean_err = mean(err2);
        out.std_err = std(err2);
        
        % For instantaneous speed with segmentation, combine segments 1 and 3
        % Calculate separately and concatenate
        if isfield(out, 'seg1_times') && isfield(out, 'seg3_times')
            % Segment 1 instantaneous speed
            t0_seg1 = out.seg1_tp(:,1);
            rp_seg1_shift = out.seg1_rp - t0_seg1;
            dim = min(3, size(rp_seg1_shift,1));
            pos_seg1 = rp_seg1_shift(1:dim, :);
            dpos_seg1 = diff(pos_seg1, 1, 2);
            ds_seg1 = sqrt(sum(dpos_seg1.^2, 1));
            dt_seg1 = diff(out.seg1_times);
            valid_seg1 = isfinite(ds_seg1) & isfinite(dt_seg1) & (dt_seg1 > 0);
            
            % Segment 3 instantaneous speed
            t0_seg3 = out.seg3_tp(:,1);
            rp_seg3_shift = out.seg3_rp - t0_seg3;
            pos_seg3 = rp_seg3_shift(1:dim, :);
            dpos_seg3 = diff(pos_seg3, 1, 2);
            ds_seg3 = sqrt(sum(dpos_seg3.^2, 1));
            dt_seg3 = diff(out.seg3_times);
            valid_seg3 = isfinite(ds_seg3) & isfinite(dt_seg3) & (dt_seg3 > 0);
            
            % Combine instantaneous speeds
            if any(valid_seg1) || any(valid_seg3)
                v_inst_seg1 = ds_seg1(valid_seg1) ./ dt_seg1(valid_seg1);
                v_inst_seg3 = ds_seg3(valid_seg3) ./ dt_seg3(valid_seg3);
                out.v_inst = [v_inst_seg1, v_inst_seg3];
                
                e_step_seg1 = out.seg1_err(2:end);
                e_step_seg3 = out.seg3_err(2:end);
                e_inst_seg1 = e_step_seg1(valid_seg1);
                e_inst_seg3 = e_step_seg3(valid_seg3);
                out.e_inst = [e_inst_seg1, e_inst_seg3];
            else
                out.v_inst = [];
                out.e_inst = [];
            end
        else
            out.v_inst = [];
            out.e_inst = [];
        end
    end

    % Store time and error arrays for plotting
    out.times = times2;
    out.errors = err2;
    
    % Calculate setting time for segment 2 if segmentation was applied
    if ~isempty(time_segments) && numel(time_segments) == 4 && isfield(out, 'seg2_err') && ~isempty(out.seg2_err)
        % Find peak error in segment 2
        [peak_err, peak_idx] = max(out.seg2_err);
        peak_time = out.seg2_times(peak_idx);
        
        % Setting time = time from peak to end of segment 2
        seg2_end_time = out.seg2_times(end);
        out.setting_time = seg2_end_time - peak_time;
        
        % Also store mean error and std for segments 1+3 combined
        if isfield(out, 'seg1_err') && isfield(out, 'seg3_err')
            out.mean_err_segments_1_3 = mean([out.seg1_err, out.seg3_err]);
            out.std_err_segments_1_3 = std([out.seg1_err, out.seg3_err]);
        end
    else
        out.setting_time = nan;
        out.mean_err_segments_1_3 = nan;
        out.std_err_segments_1_3 = nan;
    end
end


