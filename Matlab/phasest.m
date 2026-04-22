
clear; close all; clc;

%file selection
filename = 'gait_data.csv';   % swap to any csv as needed

if ~exist(filename, 'file')
    error('File "%s" not found in current folder.', filename);
end

data = readmatrix(filename);
fprintf('Loaded %s  ->  %d rows\n', filename, size(data,1));

%detect columns
t_ms  = data(:,1);
angle = data(:,2);   % degrees (complementary filter output from ESP32)
% Column 3 (raw gyro velocity) intentionally NOT used — clips at +/-250 deg/s

t  = (t_ms - t_ms(1)) / 1000;   
fs = 1 / mean(diff(t));          %actual sample rate (~43 Hz)

fprintf('Duration : %.2f s  |  Sample rate : %.1f Hz\n', t(end), fs);

%%low pass filter on angle 
fc       = 4;                              
[b, a]   = butter(4, fc/(fs/2), 'low');
angle_f  = filtfilt(b, a, angle);         

%%velocity reconstruction from filtered angle 

vel_rec          = zeros(size(angle_f));
vel_rec(2:end-1) = (angle_f(3:end) - angle_f(1:end-2)) ./ ...
                   (t(3:end)        - t(1:end-2));
vel_rec(1)       = vel_rec(2);       % edge padding
vel_rec(end)     = vel_rec(end-1);

vel_f = filtfilt(b, a, vel_rec);     %light second pass to remove diff noise

fprintf('Reconstructed vel: [%.1f , %.1f] deg/s  (no clipping)\n', ...
        min(vel_f), max(vel_f));

%%remove dc bias from Mpu6050 and normalise 

angle_c = angle_f - mean(angle_f);
vel_c   = vel_f   - mean(vel_f);

angle_n = angle_c / max(abs(angle_c));
vel_n   = vel_c   / max(abs(vel_c));


phase_raw = atan2(vel_n, angle_n);   % wrapped [-pi, pi]

autoalign
search_end        = min(round(fs * 3), length(vel_n));
[~, peak_idx]     = max(vel_n(1:search_end));
offset            = phase_raw(peak_idx);

phase_aligned = atan2(sin(phase_raw - offset), ...
                       cos(phase_raw - offset));  

%% phase velocity and prediction
phase_unwrapped = unwrap(phase_aligned);              % increasing
p_vel           = gradient(phase_unwrapped, mean(diff(t)));  

%  Low-pass smooth the phase velocity before predicting
[bp, ap] = butter(2, 2/(fs/2), 'low');         % 2 Hz
p_vel_f  = filtfilt(bp, ap, p_vel);

%  Predict 75 ms ahead
phase_pred = atan2(sin(phase_aligned + p_vel_f * 0.075), ...
                   cos(phase_aligned + p_vel_f * 0.075));

%%Stage classification
n      = length(t);
events = zeros(n, 1);

% 1: Heel Strike, 2: Mid Stance, 3: Push Off, 4: Swing
for i = 1:n
    p = phase_aligned(i);
    if     p >= -pi   && p < -pi/2,  events(i) = 1;   % Heel Strike (Red)
    elseif p >= -pi/2 && p < 0,      events(i) = 2;   % Mid Stance (Green)
    elseif p >= 0     && p < pi/2,   events(i) = 3;   % Push Off (Blue)
    else,                            events(i) = 4;   % Swing (Grey)
    end
end

stage_names  = {'Heel Strike','Mid Stance','Push Off','Swing'};
stage_colors = [1.0 0.0 0.0;    % red   - Heel Strike
                0.2 0.8 0.2;    % green - Mid Stance
                0.0 0.0 1.0;    % blue  - Push Off
                0.5 0.5 0.5];   % grey  - Swing

n      = length(t);
events = zeros(n, 1);

%% plots
figure('Name', ['Gait Analysis v3 — ' filename], 'Color', 'w', 'Position', [40 40 1150 900]);

%kinematics plot
subplot(3,2,[1 2]); yyaxis left; plot(t, angle_f, 'r'); yyaxis right; plot(t, vel_f); grid on;

%phase potrait
subplot(3,2,3); hold on;
% Plot the whole loop in a light grey first to ensure connectivity
plot(angle_n, vel_n, 'Color', [0.9 0.9 0.9], 'LineWidth', 0.5, 'HandleVisibility', 'off');

for s = 1:4
    idx = (events == s);
    % Using scatter for the portrait creates a beautiful "density" map 
    % while keeping the stages clearly separated.
    scatter(angle_n(idx), vel_n(idx), 4, stage_colors(s,:), 'filled', 'DisplayName', stage_names{s});
end
xlabel('Normalized Angle'); ylabel('Normalized Velocity');
title('Phase Portrait (Colored by Gait Stage)');
axis equal; grid on; legend('Location', 'best', 'FontSize', 8);

%phase over time
subplot(3,2,4)
phase_deg = rad2deg(phase_aligned) + 180;
pred_deg  = rad2deg(phase_pred)    + 180;
plot(t, phase_deg, 'g', 'LineWidth', 1.2); hold on;
plot(t, pred_deg,  'k--', 'LineWidth', 0.8);
ylim([0 360]); title('Gait Phase & 75ms Prediction'); grid on;

%timeline plot
subplot(3,2,[5 6]); hold on;
% Draw the "staircase" line
plot(t, events, 'k-', 'LineWidth', 0.5, 'HandleVisibility', 'off');

for s = 1:4
    idx = (events == s);
    if any(idx)
        plot(t(idx), events(idx), '.', 'Color', stage_colors(s,:), 'MarkerSize', 10, 'DisplayName', stage_names{s});
    end
end
ylim([0.5 4.5]); yticks(1:4); yticklabels(stage_names);
xlabel('Time (s)'); title('Gait Stage Timeline');
grid on; legend('Location', 'southoutside', 'Orientation', 'horizontal');

%status
fprintf('\n=== GAIT SUMMARY (%s) ===\n', filename);
for s = 1:4
    fprintf('  %-14s : %5.1f%%\n', stage_names{s}, sum(events==s)/n*100);
end
hs_edges = sum(diff(events == 3) == 1);
fprintf('  Heel strikes  : %d\n', hs_edges);
if hs_edges > 1
    step_t = t(end) / hs_edges;
    fprintf('  Avg step time : %.2f s  (%.0f steps/min)\n', ...
            step_t, 60/step_t);
end
fprintf('  LPF (angle)   : %d Hz  |  LPF (phase vel) : 2 Hz\n', fc);
fprintf('  Phase offset  : %.3f rad (auto-aligned)\n', offset);