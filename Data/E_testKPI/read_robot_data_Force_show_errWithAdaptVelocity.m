clear; clc;close all;

% === Path to your file ===
% log_file = 'test error des with imp change/Force.txt';
% log_file = '1 change coverage rate/Force.txt';
% log_file = '2 change coverage rate to 60/Force.txt';
% log_file = '3 use right err and controller/3/Force.txt';
log_file = '5 also add adaptive velocity/2/Force.txt';


outPrefix = 'Force_parsed';
[data, T] = parseForceLog(log_file, outPrefix);

realforce=data.real_force_filtered(2:end,:);
targetforce= data.desired_force(2:end,:);
adapitveVel= data.adaptive_velocity(2:end,:);
time=data.time(2:end,:)-data.time(2,:);

forceErr=realforce-targetforce;

% Define modern color scheme
colors = struct();
colors.blue = [0.2 0.4 0.8];      % X direction
colors.red = [0.8 0.2 0.2];       % Y direction  
colors.green = [0.2 0.7 0.3];     % Z direction
colors.orange = [0.9 0.5 0.1];    % Error
colors.purple = [0.6 0.3 0.7];    % Additional

% Create figure with better size and background
fig = figure('Position', [100, 100, 1200, 900], 'Color', 'white');

% --- Subplot 1: Real Force X and Z ---
subplot(3,1,1);
h1 = plot(time, realforce(:,1), 'Color', colors.blue, 'LineWidth', 2.5, 'DisplayName', 'Force X');
hold on;
h2 = plot(time, realforce(:,3), 'Color', colors.green, 'LineWidth', 2.5, 'DisplayName', 'Force Z');
grid on;
grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.15);
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Force (N)', 'FontSize', 12, 'FontWeight', 'bold');
title('Real Force Components (X and Z)', 'FontSize', 14, 'FontWeight', 'bold');
legend([h1, h2], 'Location', 'best', 'FontSize', 11, 'Box', 'on');
set(gca, 'FontSize', 11, 'LineWidth', 1.2);
xlim([min(time), max(time)]);
hold off;

% --- Subplot 2: Force Error Z ---
subplot(3,1,2);
h3 = plot(time, forceErr(:,3), 'Color', colors.orange, 'LineWidth', 2.5);
grid on;
grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.15);
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Force Error (N)', 'FontSize', 12, 'FontWeight', 'bold');
title('Force Error in Z-Direction', 'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'FontSize', 11, 'LineWidth', 1.2);
xlim([min(time), max(time)]);
% Add zero reference line (compatible with older MATLAB versions)
hold on;
plot([min(time), max(time)], [0, 0], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
hold off;

% --- Subplot 3: Adaptive Velocity ---
subplot(3,1,3);
h4 = plot(time, adapitveVel(:,1), 'Color', colors.blue, 'LineWidth', 2.5, 'DisplayName', 'V_x');
hold on;
h5 = plot(time, adapitveVel(:,2), 'Color', colors.red, 'LineWidth', 2.5, 'DisplayName', 'V_y');
h6 = plot(time, adapitveVel(:,3), 'Color', colors.green, 'LineWidth', 2.5, 'DisplayName', 'V_z');
grid on;
grid minor;
set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.15);
xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Adaptive Velocity (m/s)', 'FontSize', 12, 'FontWeight', 'bold');
title('Adaptive Velocity Components', 'FontSize', 14, 'FontWeight', 'bold');
legend([h4, h5, h6], 'Location', 'best', 'FontSize', 11, 'Box', 'on');
set(gca, 'FontSize', 11, 'LineWidth', 1.2);
xlim([min(time), max(time)]);
% Add zero reference line (compatible with older MATLAB versions)
plot([min(time), max(time)], [0, 0], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
hold off;

% Adjust subplot spacing for better appearance
% Only set properties for axes objects, not legends
allChildren = fig.Children;
for i = 1:length(allChildren)
    if isa(allChildren(i), 'matlab.graphics.axis.Axes')
        set(allChildren(i), 'TickDir', 'out', 'Box', 'on');
    end
end



%% parse_force_log.m
% Usage:
%   infile = 'Force.txt';
%   outPrefix = 'Force_parsed';
%   [data, T] = parseForceLog(infile, outPrefix);

function [data, T] = parseForceLog(infile, outPrefix)

    if nargin < 2 || isempty(outPrefix)
        [p, n] = fileparts(infile);
        outPrefix = fullfile(p, [n '_parsed']);
    end

    fid = fopen(infile, 'r');
    assert(fid > 0, 'Failed to open file: %s', infile);
    c = onCleanup(@() fclose(fid));

    % Storage (grow-by-chunk)
    time = [];
    rff  = []; % real_force_filtered_ (Nx3)
    df   = []; % desired_force_ (Nx3)
    tp   = []; % target_pose_ (Nx3)
    rp   = []; % real_pose_ (Nx3)
    eigv = []; % eig_value (Nx2 or more; here we store first 2 if present)
    av   = []; % adaptive_velocity (Nx3)

    % Current record
    cur = initRecord();

    while true
        line = fgetl(fid);
        if ~ischar(line)
            % EOF: flush last record if valid
            if cur.hasAny
                [time, rff, df, tp, rp, eigv, av] = appendRecord(time, rff, df, tp, rp, eigv, av, cur);
            end
            break;
        end

        line = strtrim(line);

        % Block delimiter
        if strcmp(line, '----------------------------------------')
            if cur.hasAny
                [time, rff, df, tp, rp, eigv, av] = appendRecord(time, rff, df, tp, rp, eigv, av, cur);
            end
            cur = initRecord();
            continue;
        end

        % Parse each known key
        if startsWith(line, 'time:')
            cur.time = parseScalarAfterColon(line);
            cur.hasAny = true;

        elseif startsWith(line, 'real_force_filtered_:')
            cur.real_force_filtered = parseVectorAfterColon(line, 3);
            cur.hasAny = true;

        elseif startsWith(line, 'desired_force_:')
            cur.desired_force = parseVectorAfterColon(line, 3);
            cur.hasAny = true;

        elseif startsWith(line, 'target_pose_:')
            cur.target_pose = parseVectorAfterColon(line, 3);
            cur.hasAny = true;

        elseif startsWith(line, 'real_pose_:')
            cur.real_pose = parseVectorAfterColon(line, 3);
            cur.hasAny = true;

        elseif startsWith(line, 'eig_value:')
            v = parseVectorAfterColon(line, NaN);
            % Keep first 2 if more are present; pad if fewer.
            cur.eig_value = padOrTrim(v(:).', 2);
            cur.hasAny = true;

        elseif startsWith(line, 'adaptive_velocity:')
            cur.adaptive_velocity = parseVectorAfterColon(line, 3);
            cur.hasAny = true;

        else
            % Unknown line: ignore (keeps robustness)
        end
    end

    % Build outputs
    data = struct();
    data.time = time(:);

    data.real_force_filtered = rff;
    data.desired_force       = df;
    data.target_pose         = tp;
    data.real_pose           = rp;
    data.eig_value           = eigv;
    data.adaptive_velocity   = av;

    % Table for CSV/export (split into component columns)
    T = table();
    T.time = data.time;

    T.rff_x = rff(:,1); T.rff_y = rff(:,2); T.rff_z = rff(:,3);
    T.df_x  = df(:,1);  T.df_y  = df(:,2);  T.df_z  = df(:,3);
    T.tp_x  = tp(:,1);  T.tp_y  = tp(:,2);  T.tp_z  = tp(:,3);
    T.rp_x  = rp(:,1);  T.rp_y  = rp(:,2);  T.rp_z  = rp(:,3);
    T.eig_1 = eigv(:,1); T.eig_2 = eigv(:,2);
    T.av_x  = av(:,1);  T.av_y  = av(:,2);  T.av_z  = av(:,3);

    % Save
    outMat = [outPrefix '.mat'];
    outCsv = [outPrefix '.csv'];
    save(outMat, 'data', 'T');
    writetable(T, outCsv);

    fprintf('Saved:\n  %s\n  %s\n', outMat, outCsv);
end

%% ---------- helpers ----------

function cur = initRecord()
    cur = struct();
    cur.time = NaN;
    cur.real_force_filtered = [NaN NaN NaN];
    cur.desired_force       = [NaN NaN NaN];
    cur.target_pose         = [NaN NaN NaN];
    cur.real_pose           = [NaN NaN NaN];
    cur.eig_value           = [NaN NaN];
    cur.adaptive_velocity   = [NaN NaN NaN];
    cur.hasAny = false;
end

function [time, rff, df, tp, rp, eigv, av] = appendRecord(time, rff, df, tp, rp, eigv, av, cur)
    time(end+1,1) = cur.time;

    rff(end+1,:)  = padOrTrim(cur.real_force_filtered, 3);
    df(end+1,:)   = padOrTrim(cur.desired_force, 3);
    tp(end+1,:)   = padOrTrim(cur.target_pose, 3);
    rp(end+1,:)   = padOrTrim(cur.real_pose, 3);
    eigv(end+1,:) = padOrTrim(cur.eig_value, 2);
    av(end+1,:)   = padOrTrim(cur.adaptive_velocity, 3);
end

function x = parseScalarAfterColon(line)
    parts = split(line, ':');
    if numel(parts) < 2, x = NaN; return; end
    x = str2double(strtrim(parts{2}));
    if isnan(x)
        % fallback for odd formatting
        nums = sscanf(line, '%*[^:]: %f');
        if ~isempty(nums), x = nums(1); end
    end
end

function v = parseVectorAfterColon(line, expectedLen)
    % Extract substring after first ':'
    k = strfind(line, ':');
    if isempty(k)
        v = nan(1, max(1, expectedLen));
        return;
    end
    s = strtrim(line(k(1)+1:end));
    nums = sscanf(s, '%f').';
    if isempty(nums)
        v = nan(1, max(1, expectedLen));
        return;
    end
    if isnan(expectedLen)
        v = nums;
    else
        v = padOrTrim(nums, expectedLen);
    end
end

function v = padOrTrim(v, n)
    v = v(:).';
    if numel(v) >= n
        v = v(1:n);
    else
        v = [v nan(1, n-numel(v))];
    end
end












