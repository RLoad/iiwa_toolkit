clear; clc; close all;

%% ===== User config =====
base_dir = '3 use right err and controller';   % 你的主目录
folder_ids = 1:12;                               % 子文件夹编号 1~9
radius = 0.07;                                  % 你的半径（用于误差定义）
use_second_target_only = true;

% If true: also plot instantaneous speed vs instantaneous error (all runs pooled)
plot_instant_scatter = true;

%% ===== Batch processing =====
avg_speed_all = nan(size(folder_ids));
mean_err_all  = nan(size(folder_ids));
total_time_all = nan(size(folder_ids));
total_dist_all = nan(size(folder_ids));

v_inst_all = [];   % pooled instantaneous speeds
e_inst_all = [];   % pooled instantaneous errors
run_id_all = [];   % which folder each point comes from (for optional coloring/analysis)

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
    total_time_all(ii) = out.total_time;
    total_dist_all(ii) = out.total_dist;

    if plot_instant_scatter && ~isempty(out.v_inst)
        v_inst_all = [v_inst_all, out.v_inst];
        e_inst_all = [e_inst_all, out.e_inst];
        run_id_all = [run_id_all, k * ones(1, numel(out.v_inst))];
    end

    fprintf('[%d] mean_err=%.6f m, avg_speed=%.6f m/s, time=%.3f s, dist=%.3f m\n', ...
        k, out.mean_err, out.avg_speed, out.total_time, out.total_dist);
end

%% ===== Plot: average speed vs mean error (per run) =====
valid_run = isfinite(avg_speed_all) & isfinite(mean_err_all);

figure('Position',[200 200 900 700]);

% subplot(2,1,1);
scatter(avg_speed_all(valid_run), mean_err_all(valid_run)*100, 80, 'filled'); grid on; hold on;
xlabel('Average speed (m/s)');
ylabel('Mean tracking error (cm)');
title('Relationship: Average Speed vs Mean Tracking Error (2nd target)');

% Add labels 1~9 on points
xs = avg_speed_all(valid_run);
ys = mean_err_all(valid_run);
ks = folder_ids(valid_run);
for i = 1:numel(xs)
    text(xs(i), ys(i), sprintf('  %d', ks(i)), 'FontSize', 10, 'VerticalAlignment', 'middle');
end

% % Optional: linear fit (only if >=2 points)
% if nnz(valid_run) >= 2
%     p = polyfit(avg_speed_all(valid_run), mean_err_all(valid_run), 1);
%     xfit = linspace(min(xs), max(xs), 100);
%     yfit = polyval(p, xfit);
%     plot(xfit, yfit, 'LineWidth', 2);
%     legend('Runs', sprintf('Linear fit: err = %.3g*v + %.3g', p(1), p(2)), 'Location', 'best');
% else
%     legend('Runs', 'Location', 'best');
% end
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

%% ===== Local function =====
function out = parse_force_file_compute_metrics(log_file, radius, second_target_only)

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

    % Shift by first target pose in this segment (translation-invariant for speed)
    t0 = tp2(:,1);
    rp2_shift = rp2 - t0;

    % Compute mean error
    out.mean_err = mean(err2);

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
end
