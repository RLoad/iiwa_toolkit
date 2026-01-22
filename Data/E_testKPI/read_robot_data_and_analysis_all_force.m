clear; clc; close all;

%% ===== User config =====
base_dir = '4 use vel give a -z dir force disturb';   % ???????????????
folder_ids = [1:14];                               % ?????????????????? 1~12
radius = 0.07;                                  % ?????????????????????????????????????????????????????????
use_second_target_only = true;

% If true: also plot instantaneous speed vs instantaneous error (all runs pooled)
plot_instant_scatter = true;

%% ===== Interactive data segmentation: Click 4 points for each experiment =====
% This section allows user to click 4 time points on each experiment's force error plot
% to segment the data into 3 segments: segment 1 (t1-t2), segment 2 (t2-t3), segment 3 (t3-t4)
click_times = cell(size(folder_ids));  % Store 4 time points for each experiment

% Check if saved click times file exists
saved_click_file = fullfile(base_dir, 'saved_click_times_force.mat');
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
    
    % Parse data to get force error for plotting
    out_temp = parse_force_file_compute_metrics(log_file, radius, use_second_target_only);
    
    if isempty(out_temp.times_force) || isempty(out_temp.desired_force_z) || isempty(out_temp.real_force_z)
        warning('No valid force data in experiment %d (skipped)', k);
        click_times{ii} = [];
        continue;
    end
    
    % Calculate force error for plotting
    force_error_z = out_temp.desired_force_z - out_temp.real_force_z;
    
    % Plot force error over time
    figure('Position', [100 100 1200 600]);
    plot(out_temp.times_force, force_error_z, 'b-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Force Error Z (N)');
    title(sprintf('Experiment %d: Click 4 points (start, seg1_end, seg2_end, end)', k), ...
          'FontSize', 12, 'FontWeight', 'bold');
    
    % Get 4 clicks from user
    fprintf('Experiment %d: Click 4 points on the plot...\n', k);
    [x_click, ~] = ginput(4);
    
    % Sort to ensure time order (t1 < t2 < t3 < t4)
    x_click = sort(x_click);
    
    % Validate that clicks are within data range
    t_min = min(out_temp.times_force);
    t_max = max(out_temp.times_force);
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
std_err_all   = nan(size(folder_ids));  % Standard deviation of force error
mean_desired_force_mag_all = nan(size(folder_ids));  % mean desired force magnitude per run
total_time_all = nan(size(folder_ids));
total_dist_all = nan(size(folder_ids));

% New arrays for segmented data analysis
avg_speed_segments_1_3 = nan(size(folder_ids));
mean_force_err_segments_1_3 = nan(size(folder_ids));
std_force_err_segments_1_3 = nan(size(folder_ids));  % Standard deviation for segments 1+3
setting_time_segment_2 = nan(size(folder_ids));

v_inst_all = [];   % pooled instantaneous speeds
e_inst_all = [];   % pooled instantaneous errors
run_id_all = [];   % which folder each point comes from (for optional coloring/analysis)

% Store force data for plotting
all_times_force = cell(numel(folder_ids), 1);
all_desired_force_z = cell(numel(folder_ids), 1);
all_real_force_z = cell(numel(folder_ids), 1);

for ii = [1:10 12:14]
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
    mean_desired_force_mag_all(ii) = out.mean_desired_force_mag;
    total_time_all(ii) = out.total_time;
    total_dist_all(ii) = out.total_dist;

    % Store segmented metrics
    if ~isempty(time_seg)
        % Use segments 1+3 for speed calculation
        avg_speed_segments_1_3(ii) = out.avg_speed;  % Already calculated from segments 1+3
        mean_force_err_segments_1_3(ii) = out.mean_err_segments_1_3;
        std_force_err_segments_1_3(ii) = out.std_err_segments_1_3;
        setting_time_segment_2(ii) = out.setting_time;
        
        % Debug output
        if isnan(avg_speed_segments_1_3(ii)) || isnan(mean_force_err_segments_1_3(ii))
            fprintf('  WARNING: Experiment %d has NaN values after segmentation\n', k);
            fprintf('    avg_speed_segments_1_3: %.6f\n', avg_speed_segments_1_3(ii));
            fprintf('    mean_err_segments_1_3: %.6f\n', mean_force_err_segments_1_3(ii));
            fprintf('    out.avg_speed: %.6f\n', out.avg_speed);
            fprintf('    out.mean_err_segments_1_3: %.6f\n', out.mean_err_segments_1_3);
        end
    else
        % If no segmentation, use original values
        avg_speed_segments_1_3(ii) = out.avg_speed;
        mean_force_err_segments_1_3(ii) = out.mean_err;
        std_force_err_segments_1_3(ii) = out.std_err;
        setting_time_segment_2(ii) = nan;
    end

    % Store force data for plotting
    all_times_force{ii} = out.times_force;
    all_desired_force_z{ii} = out.desired_force_z;
    all_real_force_z{ii} = out.real_force_z;

    if plot_instant_scatter && ~isempty(out.v_inst)
        v_inst_all = [v_inst_all, out.v_inst];
        e_inst_all = [e_inst_all, out.e_inst];
        run_id_all = [run_id_all, k * ones(1, numel(out.v_inst))];
    end

    fprintf('[%d] mean_force_err=%.6f N, std_force_err=%.6f N, avg_speed=%.6f m/s, time=%.3f s, dist=%.3f m', ...
        k, out.mean_err, out.std_err, out.avg_speed, out.total_time, out.total_dist);
    if ~isempty(time_seg)
        fprintf(', seg1+3_err=%.6f N (std=%.6f N), setting_time=%.3f s', ...
            mean_force_err_segments_1_3(ii), std_force_err_segments_1_3(ii), setting_time_segment_2(ii));
    end
    fprintf('\n');
end

%% ===== Plot: average speed vs mean force error (per run) =====
% valid_run ????????????????????? NaN ??? Inf ??????????????????????????????
% ?????????????????????????????????????????? valid_run???????????????
% ????????? valid_run ???????????? NaN/Inf ??????????????????

fprintf('\n===== Plotting: Average Speed vs Mean Force Error =====\n');

% ????????????????????????????????????????????????????????????
% ????????????????????????????????????
has_seg_data = any(isfinite(avg_speed_segments_1_3)) && any(isfinite(mean_force_err_segments_1_3));
has_orig_data = any(isfinite(avg_speed_all)) && any(isfinite(mean_err_all));

if has_seg_data
    % ??????????????????
    plot_speed = avg_speed_segments_1_3;
    plot_err = mean_force_err_segments_1_3;
    plot_err_std = std_force_err_segments_1_3;
    plot_title = 'Force tracking error in 12 trails without consider disturbance phase (Mean + Std)';
    fprintf('Using segmented data\n');
elseif has_orig_data
    % ??????????????????
    plot_speed = avg_speed_all;
    plot_err = mean_err_all;
    plot_err_std = std_err_all;
    plot_title = 'Relationship: Average Speed vs Mean Force Error (All Data, Mean + Std)';
    fprintf('Using original data (no segmented data available)\n');
else
    plot_speed = [];
    plot_err = [];
    plot_err_std = [];
    plot_title = 'Relationship: Average Speed vs Mean Force Error';
    fprintf('WARNING: No valid data available!\n');
end

figure('Position',[200 200 900 700]);

if ~isempty(plot_speed) && ~isempty(plot_err) && ~isempty(plot_err_std)
    % ?????? errorbar ?????? mean ?? std
    valid_idx = isfinite(plot_speed) & isfinite(plot_err) & isfinite(plot_err_std);
    xs = plot_speed(valid_idx);
    ys_mean = plot_err(valid_idx);
    ys_std = plot_err_std(valid_idx);
    
    errorbar(xs, ys_mean, ys_std, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'b', ...
             'LineWidth', 1.5, 'CapSize', 8); 
    grid on; hold on;
    xlabel('Average speed (m/s)');
    ylabel('Mean force error (N)');
    title(plot_title);

    % ?????????????????????????????????????????????
%     valid_ids = folder_ids(valid_idx);
%     for i = 1:numel(xs)
%         text(xs(i), ys_mean(i), sprintf('  %d', valid_ids(i)), ...
%              'FontSize', 10, 'VerticalAlignment', 'middle');
%     end
    hold off;
    fprintf('Plotted data for %d experiments\n', numel(xs));
else
    text(0.5, 0.5, 'No data to plot', ...
         'HorizontalAlignment', 'center', 'Units', 'normalized', ...
         'FontSize', 12);
    xlabel('Average speed (m/s)');
    ylabel('Mean force error (N)');
    title(plot_title);
    grid on;
end

%% ===== Plot: average speed vs setting time (per run) - Using Segment 2 =====
% ???????????????????????????????????????????????????2????????????
% ????????? valid_run??????????????????scatter ??????????????? NaN ???

fprintf('\n===== Plotting: Average Speed vs Setting Time =====\n');
fprintf('This plot requires segmentation data\n');

figure('Position',[200 200 900 700]);

% ???????????????????????????scatter ??????????????? NaN
scatter(avg_speed_segments_1_3, setting_time_segment_2, 80, 'filled'); grid on; hold on;
xlabel('Average speed (m/s)');
ylabel('Setting Time (s)');
title('');

% ?????????????????????????????????????????????
% for i = 1:numel(avg_speed_segments_1_3)
%     if isfinite(avg_speed_segments_1_3(i)) && isfinite(setting_time_segment_2(i))
%         text(avg_speed_segments_1_3(i), setting_time_segment_2(i), sprintf('  %d', folder_ids(i)), ...
%              'FontSize', 10, 'VerticalAlignment', 'middle');
%     end
% end
hold off;

% ?????????????????????
valid_count = nnz(isfinite(avg_speed_segments_1_3) & isfinite(setting_time_segment_2));
fprintf('Plotted %d valid data points out of %d experiments\n', valid_count, numel(folder_ids));

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
    h_legend = legend('show');
    set(h_legend, 'Location', 'best', 'FontSize', 7);
    hold off;
end

% Hide unused subplots
for plot_idx = (num_to_plot + 1):num_subplots
    % subplot(4, 4, plot_idx);
    axis off;
end

% Add title for all subplots (compatible with MATLAB 2017)
ha = axes('Position', [0 0 1 1], 'Visible', 'off');
text(0.5, 0.98, 'Desired Force (Z) vs Real Force (Z) Over Time - All Experiments', ...
     'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
     'FontSize', 14, 'FontWeight', 'bold', 'Parent', ha);

%% ===== Interactive: Plot detailed analysis for selected experiment =====
fprintf('\n===== Detailed Analysis: Select an experiment to plot =====\n');
fprintf('Available experiments: ');
valid_exp_list = [];
for ii = 1:numel(folder_ids)
    k = folder_ids(ii);
    log_file = fullfile(base_dir, num2str(k), 'Force.txt');
    if exist(log_file, 'file') == 2
        valid_exp_list(end+1) = k;
        fprintf('%d ', k);
    end
end
fprintf('\n');

if ~isempty(valid_exp_list)
    selected_exp = input('Enter experiment number to plot detailed analysis (or press Enter to skip): ');
    
    if ~isempty(selected_exp) && ismember(selected_exp, valid_exp_list)
        % Parse detailed data for selected experiment
        log_file = fullfile(base_dir, num2str(selected_exp), 'Force.txt');
        detailed_data = parse_detailed_force_data(log_file, radius, use_second_target_only);
        
        if ~isempty(detailed_data.times_force) && ~isempty(detailed_data.force_error_z)
            % Create figure with 3 subplots
            figure('Position', [100 100 1400 900]);
            
            % Subplot 1: Force tracking error (Z-direction)
            subplot(3, 1, 1);
            plot(detailed_data.times_force, detailed_data.force_error_z, 'r-', 'LineWidth', 1.5);
            grid on;
            xlabel('Time (s)');
            ylabel('Force Error Z (N)');
            title(sprintf('Force Tracking Error (Z-direction) Over Time - Experiment %d', selected_exp));
            
            % Subplot 2: Speed over time
            subplot(3, 1, 2);
            if ~isempty(detailed_data.speeds)
                plot(detailed_data.times_speed, detailed_data.speeds, 'b-', 'LineWidth', 1.5);
                grid on;
                xlabel('Time (s)');
                ylabel('Speed (m/s)');
                title(sprintf('Speed Over Time - Experiment %d', selected_exp));
            else
                text(0.5, 0.5, 'No speed data available', ...
                     'HorizontalAlignment', 'center', 'Units', 'normalized', ...
                     'FontSize', 12);
                grid on;
                xlabel('Time (s)');
                ylabel('Speed (m/s)');
                title(sprintf('Speed Over Time - Experiment %d', selected_exp));
            end
            
            % Subplot 3: Impedance parameters (eigenvalues)
            subplot(3, 1, 3);
            if ~isempty(detailed_data.eigvals) && size(detailed_data.eigvals, 1) > 0
                num_eig = size(detailed_data.eigvals, 1);
                colors = lines(num_eig);
                for i = 1:num_eig
                    plot(detailed_data.times_eig, detailed_data.eigvals(i,:), '-', ...
                         'LineWidth', 1.5, 'Color', colors(i,:), ...
                         'DisplayName', sprintf('Eigenvalue %d', i));
                    hold on;
                end
                hold off;
                legend('Location', 'best');
            else
                text(0.5, 0.5, 'No impedance data available', ...
                     'HorizontalAlignment', 'center', 'Units', 'normalized', ...
                     'FontSize', 12);
            end
            grid on;
            xlabel('Time (s)');
            ylabel('Impedance Parameter (Eigenvalue)');
            title(sprintf('Impedance Parameters Over Time - Experiment %d', selected_exp));
            
            % Add title for all subplots (compatible with MATLAB 2017)
            ha_detailed = axes('Position', [0 0 1 1], 'Visible', 'off');
            text(0.5, 0.98, sprintf('Detailed Analysis - Experiment %d', selected_exp), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
                 'FontSize', 14, 'FontWeight', 'bold', 'Parent', ha_detailed);
            % Original sgtitle call (commented out for compatibility):
            % sgtitle(sprintf('Detailed Analysis - Experiment %d', selected_exp), ...
            %         'FontSize', 14, 'FontWeight', 'bold');
        else
            warning('No valid data found for experiment %d', selected_exp);
        end
    else
        fprintf('Invalid experiment number or skipped.\n');
    end
else
    fprintf('No valid experiments found.\n');
end

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
    
    % Normalize force times (must be done before segmentation)
    if ~isempty(times_force2)
        times_force2 = times_force2 - times_force2(1);
    end

    % Initialize output struct
    out = struct();
    segmentation_applied = false;

    % Apply time segmentation if provided
    if ~isempty(time_segments) && numel(time_segments) == 4
        t1 = time_segments(1);
        t2 = time_segments(2);
        t3 = time_segments(3);
        t4 = time_segments(4);
        
        % Find indices corresponding to the time points (use normalized force times)
        idx_t1 = find(times_force2 >= t1, 1, 'first');
        idx_t2 = find(times_force2 >= t2, 1, 'first');
        idx_t3 = find(times_force2 >= t3, 1, 'first');
        idx_t4 = find(times_force2 >= t4, 1, 'first');
        
        fprintf('    Segmentation time points: t1=%.3f, t2=%.3f, t3=%.3f, t4=%.3f\n', t1, t2, t3, t4);
        fprintf('    Force time range: [%.3f, %.3f], data points: %d\n', ...
            min(times_force2), max(times_force2), numel(times_force2));
        fprintf('    Found indices: idx_t1=%d, idx_t2=%d, idx_t3=%d, idx_t4=%d\n', ...
            idx_t1, idx_t2, idx_t3, idx_t4);
        
        if isempty(idx_t1) || isempty(idx_t4)
            warning('Time segmentation points out of range, using all data');
            fprintf('    ERROR: Cannot find time points in data range!\n');
        else
            % Clip to valid range
            idx_t1 = max(1, idx_t1);
            idx_t2 = max(idx_t1, min(idx_t2, numel(times_force2)));
            idx_t3 = max(idx_t2, min(idx_t3, numel(times_force2)));
            idx_t4 = min(numel(times_force2), idx_t4);
            
            % Find corresponding indices in position data (times2)
            % IMPORTANT: times2 needs to be normalized first if not already
            if ~isempty(times2) && times2(1) ~= 0
                times2_normalized = times2 - times2(1);
            else
                times2_normalized = times2;
            end
            
            % Map time points to position indices
            idx_t1_pos = find(times2_normalized >= t1, 1, 'first');
            idx_t2_pos = find(times2_normalized >= t2, 1, 'first');
            idx_t3_pos = find(times2_normalized >= t3, 1, 'first');
            idx_t4_pos = find(times2_normalized >= t4, 1, 'first');
            
            fprintf('    Position time range: [%.3f, %.3f], data points: %d\n', ...
                min(times2_normalized), max(times2_normalized), numel(times2_normalized));
            fprintf('    Found position indices: idx_t1_pos=%d, idx_t2_pos=%d, idx_t3_pos=%d, idx_t4_pos=%d\n', ...
                idx_t1_pos, idx_t2_pos, idx_t3_pos, idx_t4_pos);
            
            if isempty(idx_t1_pos), idx_t1_pos = 1; end
            if isempty(idx_t2_pos), idx_t2_pos = numel(times2); end
            if isempty(idx_t3_pos), idx_t3_pos = numel(times2); end
            if isempty(idx_t4_pos), idx_t4_pos = numel(times2); end
            
            idx_t1_pos = max(1, idx_t1_pos);
            idx_t2_pos = max(idx_t1_pos, min(idx_t2_pos, numel(times2)));
            idx_t3_pos = max(idx_t2_pos, min(idx_t3_pos, numel(times2)));
            idx_t4_pos = min(numel(times2), idx_t4_pos);
            
            % Check if segments have enough data points
            if idx_t2_pos - idx_t1_pos < 1 || idx_t4_pos - idx_t3_pos < 1
                warning('Segments 1 or 3 have insufficient data points for speed calculation');
                fprintf('    Segment 1: %d points, Segment 3: %d points\n', ...
                    idx_t2_pos - idx_t1_pos + 1, idx_t4_pos - idx_t3_pos + 1);
            end
            
            % Extract segments for force data
            seg1_times_force = times_force2(idx_t1:idx_t2);
            seg1_desired_z = desired_force_z2(idx_t1:idx_t2);
            seg1_real_z = real_force_z2(idx_t1:idx_t2);
            seg1_err_force = seg1_desired_z - seg1_real_z;
            
            seg2_times_force = times_force2(idx_t2:idx_t3);
            seg2_desired_z = desired_force_z2(idx_t2:idx_t3);
            seg2_real_z = real_force_z2(idx_t2:idx_t3);
            seg2_err_force = seg2_desired_z - seg2_real_z;
            
            seg3_times_force = times_force2(idx_t3:idx_t4);
            seg3_desired_z = desired_force_z2(idx_t3:idx_t4);
            seg3_real_z = real_force_z2(idx_t3:idx_t4);
            seg3_err_force = seg3_desired_z - seg3_real_z;
            
            % Extract segments for position data
            seg1_times = times2(idx_t1_pos:idx_t2_pos);
            seg1_tp = tp2(:, idx_t1_pos:idx_t2_pos);
            seg1_rp = rp2(:, idx_t1_pos:idx_t2_pos);
            
            seg2_times = times2(idx_t2_pos:idx_t3_pos);
            seg2_tp = tp2(:, idx_t2_pos:idx_t3_pos);
            seg2_rp = rp2(:, idx_t2_pos:idx_t3_pos);
            
            seg3_times = times2(idx_t3_pos:idx_t4_pos);
            seg3_tp = tp2(:, idx_t3_pos:idx_t4_pos);
            seg3_rp = rp2(:, idx_t3_pos:idx_t4_pos);
            
            % Store segment information in output
            out.seg1_times_force = seg1_times_force;
            out.seg1_err_force = seg1_err_force;
            out.seg2_times_force = seg2_times_force;
            out.seg2_err_force = seg2_err_force;
            out.seg3_times_force = seg3_times_force;
            out.seg3_err_force = seg3_err_force;
            
            out.seg1_times = seg1_times;
            out.seg1_tp = seg1_tp;
            out.seg1_rp = seg1_rp;
            out.seg2_times = seg2_times;
            out.seg2_tp = seg2_tp;
            out.seg2_rp = seg2_rp;
            out.seg3_times = seg3_times;
            out.seg3_tp = seg3_tp;
            out.seg3_rp = seg3_rp;
            
            % For combined segments 1+3 (used for error calculation)
            seg1_3_times_force = [seg1_times_force, seg3_times_force];
            seg1_3_desired_z = [seg1_desired_z, seg3_desired_z];
            seg1_3_real_z = [seg1_real_z, seg3_real_z];
            seg1_3_err_force = [seg1_err_force, seg3_err_force];
            
            seg1_3_times = [seg1_times, seg3_times];
            seg1_3_tp = [seg1_tp, seg3_tp];
            seg1_3_rp = [seg1_rp, seg3_rp];
            
            % Use segments 1+3 for main calculations
            times_force2 = seg1_3_times_force;
            desired_force_z2 = seg1_3_desired_z;
            real_force_z2 = seg1_3_real_z;
            err2 = seg1_3_err_force;  % This is force error for segments 1+3
            
            times2 = seg1_3_times;
            tp2 = seg1_3_tp;
            rp2 = seg1_3_rp;
            
            % Debug: check segment sizes
            fprintf('    Segmentation: seg1=%d points, seg2=%d points, seg3=%d points\n', ...
                numel(seg1_times_force), numel(seg2_times_force), numel(seg3_times_force));
            fprintf('    Combined seg1+3: %d force points, %d position points\n', ...
                numel(seg1_3_err_force), numel(seg1_3_times));
            
            % Calculate speed separately for segments 1 and 3 to avoid position discontinuity
            % Check if segments have data
            if size(seg1_rp, 2) < 2 || size(seg3_rp, 2) < 2
                warning('Segments 1 or 3 have insufficient position data points');
                fprintf('    Segment 1: %d points, Segment 3: %d points\n', size(seg1_rp, 2), size(seg3_rp, 2));
                out.avg_speed = nan;
                segmentation_applied = true;
            else
                % Segment 1 speed calculation
                t0_seg1 = seg1_tp(:,1);
                rp_seg1_shift = seg1_rp - t0_seg1;
                dim = min(3, size(rp_seg1_shift,1));
                pos_seg1 = rp_seg1_shift(1:dim, :);
                
                if size(pos_seg1, 2) < 2
                    dist_seg1 = 0;
                    time_seg1 = 0;
                else
                    dpos_seg1 = diff(pos_seg1, 1, 2);
                    ds_seg1 = sqrt(sum(dpos_seg1.^2, 1));
                    dt_seg1 = diff(seg1_times);
                    valid_seg1 = isfinite(ds_seg1) & isfinite(dt_seg1) & (dt_seg1 > 0);
                    dist_seg1 = sum(ds_seg1(valid_seg1));
                    time_seg1 = sum(dt_seg1(valid_seg1));
                end
                
                % Segment 3 speed calculation
                t0_seg3 = seg3_tp(:,1);
                rp_seg3_shift = seg3_rp - t0_seg3;
                pos_seg3 = rp_seg3_shift(1:dim, :);
                
                if size(pos_seg3, 2) < 2
                    dist_seg3 = 0;
                    time_seg3 = 0;
                else
                    dpos_seg3 = diff(pos_seg3, 1, 2);
                    ds_seg3 = sqrt(sum(dpos_seg3.^2, 1));
                    dt_seg3 = diff(seg3_times);
                    valid_seg3 = isfinite(ds_seg3) & isfinite(dt_seg3) & (dt_seg3 > 0);
                    dist_seg3 = sum(ds_seg3(valid_seg3));
                    time_seg3 = sum(dt_seg3(valid_seg3));
                end
                
                % Combined speed for segments 1+3
                out.total_dist = dist_seg1 + dist_seg3;
                out.total_time = time_seg1 + time_seg3;
                fprintf('    Speed calculation: dist_seg1=%.6f, dist_seg3=%.6f, time_seg1=%.6f, time_seg3=%.6f\n', ...
                    dist_seg1, dist_seg3, time_seg1, time_seg3);
                fprintf('    Segment 1: %d pos points, Segment 3: %d pos points\n', ...
                    size(seg1_rp, 2), size(seg3_rp, 2));
                
                if out.total_time > 0
                    out.avg_speed = out.total_dist / out.total_time;
                    fprintf('    Calculated avg_speed: %.6f m/s\n', out.avg_speed);
                else
                    out.avg_speed = nan;
                    warning('Total time is zero or negative for segments 1+3 (dist=%.6f, time=%.6f)', ...
                        out.total_dist, out.total_time);
                end
                segmentation_applied = true;
            end
        end
    end
    
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
    
    % Force times are already normalized above (before segmentation)

    % If no segmentation, use original calculation
    if ~segmentation_applied
        % Shift by first target pose in this segment (translation-invariant for speed)
        t0 = tp2(:,1);
        rp2_shift = rp2 - t0;

        % Compute mean error and standard deviation (force error)
        valid_err2 = err2(~isnan(err2));
        out.mean_err = mean(valid_err2);
        out.std_err = std(valid_err2);

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
    else
        % Mean error and standard deviation for segments 1+3
        valid_err2 = err2(~isnan(err2));
        out.mean_err = mean(valid_err2);
        out.std_err = std(valid_err2);
    end

    % Instantaneous speed vs instantaneous error (aligned to step i: between i and i+1)
    if ~segmentation_applied
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
        % For instantaneous speed with segmentation, combine segments 1 and 3
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
                
                e_step_seg1 = out.seg1_err_force(2:end);
                e_step_seg3 = out.seg3_err_force(2:end);
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
    
    % Store force data for plotting
    out.times_force = times_force2;
    out.desired_force_z = desired_force_z2;
    out.real_force_z = real_force_z2;
    
    % Calculate setting time for segment 2 and mean error for segments 1+3 if segmentation was applied
    if segmentation_applied
        % Mean error and standard deviation for segments 1+3 (err2 already contains seg1+seg3 combined)
        if ~isempty(err2)
            valid_err = err2(~isnan(err2));
            if ~isempty(valid_err)
                out.mean_err_segments_1_3 = mean(valid_err);
                out.std_err_segments_1_3 = std(valid_err);
            else
                out.mean_err_segments_1_3 = nan;
                out.std_err_segments_1_3 = nan;
                warning('No valid error data in segments 1+3 after segmentation');
            end
        else
            out.mean_err_segments_1_3 = nan;
            out.std_err_segments_1_3 = nan;
            warning('err2 is empty after segmentation');
        end
        
        % Setting time for segment 2
        if isfield(out, 'seg2_err_force') && ~isempty(out.seg2_err_force) && numel(out.seg2_err_force) > 0
            valid_seg2_err = out.seg2_err_force(~isnan(out.seg2_err_force));
            if ~isempty(valid_seg2_err) && isfield(out, 'seg2_times_force') && ~isempty(out.seg2_times_force)
                [~, peak_idx] = max(abs(valid_seg2_err));
                % Find the original index in seg2_err_force
                seg2_valid_mask = ~isnan(out.seg2_err_force);
                seg2_valid_indices = find(seg2_valid_mask);
                if peak_idx <= numel(seg2_valid_indices)
                    peak_idx_orig = seg2_valid_indices(peak_idx);
                    if peak_idx_orig <= numel(out.seg2_times_force)
                        peak_time = out.seg2_times_force(peak_idx_orig);
                        seg2_end_time = out.seg2_times_force(end);
                        out.setting_time = seg2_end_time - peak_time;
                    else
                        out.setting_time = nan;
                    end
                else
                    out.setting_time = nan;
                end
            else
                out.setting_time = nan;
            end
        else
            out.setting_time = nan;
        end
    else
        out.setting_time = nan;
        out.mean_err_segments_1_3 = nan;
        out.std_err_segments_1_3 = nan;
    end
end

%% ===== Function to parse detailed force data for plotting =====
function data = parse_detailed_force_data(log_file, radius, second_target_only)
    % Parse detailed data including force error, speed, and impedance parameters
    
    fid = fopen(log_file, 'r');
    if fid < 0
        error('Cannot open file: %s', log_file);
    end

    times = [];
    times_force = [];
    tp_all = [];
    rp_all = [];
    desired_force_z_all = [];
    real_force_z_all = [];
    eigvals = [];
    target_history = [];

    target_pose = [];
    real_force_filtered = [];
    desired_force = [];
    current_time = [];
    current_eigval = [];

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

            % store position data
            rp_all(:,end+1) = real_pose;
            tp_all(:,end+1) = target_pose;
            
            % Calculate z-direction force error and store force data
            if ~isempty(real_force_filtered) && ~isempty(desired_force)
                if length(real_force_filtered) >= 3 && length(desired_force) >= 3
                    desired_force_z_all(end+1) = desired_force(3);
                    real_force_z_all(end+1) = real_force_filtered(3);
                    times_force(end+1) = current_time;
                end
            end
            
            % Store eigenvalue if available (from previous line)
            if ~isempty(current_eigval)
                if isempty(eigvals)
                    eigvals = current_eigval(:);
                else
                    eigvals(:,end+1) = current_eigval(:);
                end
                current_eigval = [];
            else
                % If no eigenvalue, add NaN to maintain alignment
                if isempty(eigvals)
                    eigvals = nan(2, 1);  % Assume 2 eigenvalues based on data format
                else
                    eigvals(:,end+1) = nan(size(eigvals, 1), 1);
                end
            end

        elseif startsWith(line,'eig_value:')
            vals = sscanf(line(length('eig_value:')+1:end),'%f');
            current_eigval = vals;
            
        elseif startsWith(line,'----------------------------------------')
            % Reset force variables at separator
            real_force_filtered = [];
            desired_force = [];
            target_pose = [];
        end
    end
    
    % Handle case where last eigenvalue is not followed by real_pose
    if ~isempty(current_eigval) && size(rp_all, 2) == size(eigvals, 2)
        eigvals(:,end+1) = current_eigval(:);
    end
    
    fclose(fid);

    % Normalize time
    if ~isempty(times)
        times = times - times(1);
    end
    if ~isempty(times_force)
        times_force = times_force - times_force(1);
    end

    % Decide segment
    if second_target_only && ~isempty(target_history)
        changes = find(any(abs(diff(target_history,1,2)) > 1e-6, 1));
        if isempty(changes)
            warning('Only one target pose detected in %s; using all data.', log_file);
            idx_start = 1;
        elseif numel(changes) == 1
            idx_start = changes(1) + 1;
        else
            idx_start = changes(2) + 1;
        end
    else
        idx_start = 1;
    end

    % Ensure eigvals alignment with pose data
    num_poses = size(rp_all, 2);
    if ~isempty(eigvals)
        if size(eigvals, 2) < num_poses
            eigvals(:, end+1:num_poses) = nan(size(eigvals, 1), num_poses - size(eigvals, 2));
        elseif size(eigvals, 2) > num_poses
            eigvals = eigvals(:, 1:num_poses);
        end
    end

    % Slice data
    if idx_start <= num_poses && idx_start <= numel(times)
        data.times = times(idx_start:end);
        data.target_pose = tp_all(:,idx_start:end);
        data.real_pose = rp_all(:,idx_start:end);
        
        % Slice force data
        if idx_start <= numel(times_force)
            data.times_force = times_force(idx_start:end);
            data.desired_force_z = desired_force_z_all(idx_start:end);
            data.real_force_z = real_force_z_all(idx_start:end);
            data.force_error_z = data.desired_force_z - data.real_force_z;
        else
            data.times_force = [];
            data.desired_force_z = [];
            data.real_force_z = [];
            data.force_error_z = [];
        end
        
        % Calculate instantaneous speeds
        if size(data.real_pose, 2) > 1
            t0 = data.target_pose(:,1);
            rp_shift = data.real_pose - t0;
            dim = min(3, size(rp_shift,1));
            pos = rp_shift(1:dim, :);
            
            dpos = diff(pos, 1, 2);
            ds = sqrt(sum(dpos.^2, 1));
            dt = diff(data.times);
            
            valid = isfinite(ds) & isfinite(dt) & (dt > 0);
            if any(valid)
                data.speeds = ds(valid) ./ dt(valid);
                data.times_speed = data.times(2:end);
                data.times_speed = data.times_speed(valid);
            else
                data.speeds = [];
                data.times_speed = [];
            end
        else
            data.speeds = [];
            data.times_speed = [];
        end
        
        % Slice eigenvalues if available
        if ~isempty(eigvals) && size(eigvals, 2) >= idx_start
            data.eigvals = eigvals(:,idx_start:end);
            data.times_eig = data.times;  % Use same time as position data
        else
            data.eigvals = [];
            data.times_eig = [];
        end
    else
        data.times = [];
        data.target_pose = [];
        data.real_pose = [];
        data.times_force = [];
        data.desired_force_z = [];
        data.real_force_z = [];
        data.force_error_z = [];
        data.speeds = [];
        data.times_speed = [];
        data.eigvals = [];
        data.times_eig = [];
    end
end
