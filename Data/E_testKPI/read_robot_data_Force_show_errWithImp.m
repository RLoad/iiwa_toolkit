clear; clc;close all;

% === Path to your file ===
% log_file = 'test error des with imp change/Force.txt';
% log_file = '1 change coverage rate/Force.txt';
% log_file = '2 change coverage rate to 60/Force.txt';
log_file = '3 use right err and controller/Force.txt';
gif_name = 'tracking_animation_second_target_zeroed.gif';

fid = fopen(log_file, 'r');

times   = [];
tp_all  = [];
rp_all  = [];
err_all = [];
eigvals = [];
target_history = [];

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
    elseif startsWith(line,'eig_value:')
        vals = sscanf(line(length('eig_value:')+1:end),'%f');
        eigvals(:,end+1) = vals;
    end
end
fclose(fid);

% Normalize time
times = times - times(1);

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
err2     = err_all(idx_start:end);
eig2     = eigvals(:,idx_start:end);

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
subplot(3,1,1);
plot(circle_x, circle_y, 'k--', 'LineWidth',1.5); hold on;
plot(tp2_shift(1,:), tp2_shift(2,:), 'kx', 'MarkerSize',8, 'LineWidth',2);
plot(rp2_shift(1,:), rp2_shift(2,:), 'b-', 'LineWidth',2);
plot(rp2_shift(1,end), rp2_shift(2,end), 'ro', 'MarkerSize',8, 'LineWidth',2);
hold off;
xlabel('X (shifted)'); ylabel('Y (shifted)');
title('Shifted Trajectory (Second Target)');
legend('Desired circle','Target(0)','Real traj','Last pose','Location','best');
axis equal; grid on;

% --- Subplot 2: Error ---
subplot(3,1,2);
plot(times2, err2,'r-','LineWidth',2);
xlabel('Time (s)'); ylabel('Error (m)');
title('Error Norm Evolution');
grid on;

% --- Subplot 3: Eigenvalues ---
subplot(3,1,3);
plot(times2, eig2','LineWidth',2);
xlabel('Time (s)'); ylabel('Eigenvalues');
title('Eigenvalue Evolution');
grid on;

sgtitle('Final Results (Second Target Only)');

%% === Ask user if we should animate ===
answer = input('Generate GIF animation? (y/n): ','s');

if ~(strcmpi(answer,'y') || strcmpi(answer,'yes'))
    disp('Animation skipped. Script finished.');
    return;
end

%% === Animation plotting ===
disp('Generating animation GIF...');
figure('Position',[200 200 900 700]);
num_frames = length(times2);

for k = 1:num_frames
    
    % ----- Subplot 1: 2D trajectory (XY) -----
    subplot(3,1,1);
    plot(circle_x, circle_y, 'k--', 'LineWidth', 1.5); hold on;
    plot(tp2_shift(1,:), tp2_shift(2,:), 'kx', 'MarkerSize',8, 'LineWidth',2);
    plot(rp2_shift(1,1:k), rp2_shift(2,1:k), 'b-', 'LineWidth',2);
    plot(rp2_shift(1,k), rp2_shift(2,k), 'ro', 'MarkerSize',8, 'LineWidth',2);
    hold off;
    xlabel('X (shifted)'); ylabel('Y (shifted)');
    title(sprintf('Shifted Trajectory (Second Target) frame %d/%d',k,num_frames));
    legend('Desired circle','Target (0)','Real traj','Current real','Location','best');
    axis equal; grid on;
    
    % ----- Subplot 2: Error -----
    subplot(3,1,2);
    plot(times2(1:k), err2(1:k),'r-','LineWidth',2);
    xlabel('Time (s)'); ylabel('Error (m)');
    title('Error Norm Evolution');
    grid on; xlim([times2(1) times2(end)]);
    
    % ----- Subplot 3: Eigenvalues -----
    subplot(3,1,3);
    plot(times2(1:k), eig2(:,1:k)','LineWidth',2);
    xlabel('Time (s)'); ylabel('Eigenvalues');
    title('Eigenvalue Evolution');
    grid on; xlim([times2(1) times2(end)]);
    
    drawnow;
    
    % ---- Save frame to GIF ----
    frame = getframe(gcf);
    [imind, cm] = rgb2ind(frame2im(frame),256);
    if k == 1
        imwrite(imind,cm,gif_name,'gif','Loopcount',inf,'DelayTime',0.05);
    else
        imwrite(imind,cm,gif_name,'gif','WriteMode','append','DelayTime',0.05);
    end
end

disp(['GIF saved as: ', gif_name]);
