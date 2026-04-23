% Starting Conditions
clear; clc;

% ===========================================================================
%  A                       SETUP INITIAL VARIABLES
% ===========================================================================
init_x = [0; 0];            % Target's true position
init_x_hat = [-1.25; 1.8];  % Target's estimated position
init_y = [-0.92; 2];        % Agent's position
T = 150;                    % Elapse time
tau = [0.001];                % Time-step duration
d_des = 0.65;

N = floor(T/tau);
t_exp = 0:N-1;
t_exp = t_exp * tau;

a1 = 0.65;                  % 0 < a1, a2 < 1s
a2 = 0.75;
Tc1 = 0.9;                  % Tc2 >= Tc1
Tc2 = 1.5;      
kw = 0.6;                   % Tangential constant

% ===========================================================================
%  B                     GENERATE IM BOOLEAN MASK
% ===========================================================================
N = floor(T/tau);
minInt = 0.65;      % The minimum intermittent time
maxInt = 1.6;       % The maximum intermittent time
grace = 0;          % The amount of time before an intermittent measurement
chance = 6/1000;
intermittentTimes = generateIntermittentMask(N, minInt, maxInt, grace, tau, chance);

% ===========================================================================
%  C          READ IM DATA FROM DATASET (Comment out if you want random generation)
% ===========================================================================
% filename = 'Datasets/EXP.csv';
% % DIFFERENT DATASETS WILL REQUIRE DIFFERENT LINES TO READ
% 
% % --- Read experiment initialisation data ---
% readLine = 3;                 % Different datasets require different values
% 
% opts = detectImportOptions(filename);
% opts.DataLines = [readLine readLine];
% init_data = readmatrix(filename, opts);
% 
% k_w  = init_data(1);
% d_des = init_data(2);
% a1   = init_data(3);
% T_c1 = init_data(4);
% a2   = init_data(5);
% T_c2 = init_data(6);
% 
% % --- Read experiment data starting from line ---
% readLine = 7;                 % Different datasets require different values
% opts = detectImportOptions(filename);
% opts.DataLines = [readLine Inf];
% data = readmatrix(filename, opts);
% 
% % --- Compute movement and trim initial hovering data ---
% pos_e = vecnorm(data(:,3:4), 2, 2);
% movement = abs(pos_e - pos_e(1));
% 
% start_index = find(movement > 0.1, 1, 'first');
% data = data(start_index:end, :);
% 
% % --- Recompute time and tau ---
% data(:,1) = data(:,1) - data(1,1);   % reset time to zero
% t_exp = data(:,1);                   % experimental time vector
% tau = [0; diff(t_exp)];              % recomputed timestep durations
% 
% % --- Extract IM mask AFTER trimming ---
% intermittentTimes = data(:,22);
% 
% % --- Initial states ---
% init_x      = [0; 0];
% init_x_hat  = [data(1,7); data(1,8)];
% init_y      = [data(1,3); data(1,4)];
% N = size(data,1);
% 
% % --- Experimental errors ---
% est_error_EXP = vecnorm(data(:,7:8), 2, 2);
% est_error_EXP= est_error_EXP';
% dist_error_EXP = vecnorm(data(:,3:4), 2, 2) - d_des;
% dist_error_EXP = dist_error_EXP';
% y_data_EXP = data(:,3:4)';

% ===========================================================================
%  D                        RUN BoTLC ALGORITHMS 
% ===========================================================================

[y_data_base, est_error_base, dist_error_base] = Sui2025(N, tau, init_x, init_x_hat, init_y, d_des, a1, a2, Tc1, Tc2, kw);
[y_data_IM, est_error_IM, dist_error_IM] = PDT_IM(N, tau, init_x, init_x_hat, init_y, d_des, intermittentTimes, a1, a2, Tc1, Tc2, kw);

% ===========================================================================
%  E        FORMAT IM DATA INTO BOTH AVAILABLE AND UNAVAILABLE SETS
% ===========================================================================

idx_available   = intermittentTimes == 1;
idx_unavailable = intermittentTimes == 0;

% AVAILABLE

y_data_available = zeros(2, N);
y_data_available(:, idx_available)    = y_data_IM(:,idx_available);
y_data_available(:, idx_unavailable)    = NaN;

est_error_available = zeros(1, N);
est_error_available(idx_available)   = est_error_IM(idx_available);
est_error_available(idx_unavailable)    = NaN;

dist_error_available = zeros(1, N);
dist_error_available(idx_available)   = dist_error_IM(idx_available);
dist_error_available(idx_unavailable)    = NaN;

% UNAVAILABLE

y_data_unavailable = zeros(2, N);
y_data_unavailable(:, idx_available)  = NaN;
y_data_unavailable(:, idx_unavailable)  = y_data_IM(:,idx_unavailable);

est_error_unavailable = zeros(1, N);
est_error_unavailable(idx_available)  = NaN;
est_error_unavailable(idx_unavailable)  = est_error_IM(idx_unavailable);

dist_error_unavailable = zeros(1, N);
dist_error_unavailable(idx_available)    = NaN;
dist_error_unavailable(idx_unavailable)  = dist_error_IM(idx_unavailable);

% ===========================================================================
%  F                           PLOT THE DATA
% ===========================================================================
myColor = linspecer(4);
figure(1); clf;

% Agent Trajectory Plot
subplot(1,3,1);
hold on;
title('Agent Trajectory');

plot(init_x(1), init_x(2), '+', 'DisplayName','Target Position', 'Color', 'black', LineWidth=1);
plot(init_y(1), init_y(2), 'o', 'DisplayName','Agent Position', 'Color', 'black', LineWidth=1);
plot(init_x_hat(1), init_x_hat(2), 'o', 'DisplayName','O(0) Position', 'Color', 'magenta', LineWidth=1.3);

% ----------- Plot agent path -----------
plot(y_data_base(1,:), y_data_base(2,:), 'DisplayName', 'Base Model', 'Color', myColor(3,:), LineWidth=1.2);
plot(y_data_unavailable(1,:), y_data_unavailable(2,:), 'DisplayName', 'Proposed Model (Unavailable)', 'Color', myColor(1,:), LineWidth=1);
plot(y_data_available(1,:), y_data_available(2,:), 'DisplayName', 'Proposed Model (Available)', 'Color', myColor(4,:), LineWidth=1.5);
% plot(data(:,3), data(:,4), 'DisplayName','Experiment', 'Color', myColor(2,:), 'LineWidth', 1);


legend();
xlim([-1.6, 1.6]);
ylim([-1.1, 2.1]);

xlabel('X-Position (m)'); 
ylabel('Y-Position (m)'); 
grid on;
hold off;

% ----------- Estimation Error Plot -----------
subplot(1,3,2);
hold on;
title('Estimation Error');

plot(t_exp, est_error_base, 'DisplayName', 'Base Model', 'Color', myColor(3,:), LineWidth=1.2);
plot(t_exp, est_error_unavailable, 'DisplayName', 'Proposed Model (Unavailable)', 'Color', myColor(1,:),  LineWidth=1);
plot(t_exp, est_error_available, 'DisplayName', 'Proposed Model (Available)', 'Color', myColor(4,:),  LineWidth=1.5);
% plot(t_exp, est_error_EXP, 'DisplayName','Experiment', 'Color', myColor(2,:), LineWidth=1.2);


legend();
xlim([-0.2, 5]);
ylim([-0.04, 2.2]);

xlabel('Time (s)'); 
ylabel('Error (m)'); 
grid on;
hold off;

% ----------- Position Error Plot -----------
subplot(1,3,3);
hold on;
title('Tracking Error');

plot(t_exp, dist_error_base, 'DisplayName', 'Base Model', 'Color', myColor(3,:), LineWidth=1.2);
plot(t_exp, dist_error_unavailable, 'DisplayName', 'Proposed Model (Unavailable)', 'Color', myColor(1,:),  LineWidth=1);
plot(t_exp, dist_error_available, 'DisplayName', 'Proposed Model (Available)', 'Color', myColor(4,:),  LineWidth=1.5);
% plot(t_exp, dist_error_EXP, 'DisplayName','Experiment', 'Color', myColor(2,:), LineWidth=1.2);


legend();
xlim([-0.2, 5]);
ylim([-0.04, 2.2]);

xlabel('Time (s)'); 
ylabel('Error'); 
grid on;
hold off;



% ===========================================================================
%  G                 VERIFICATION OF TIMESTEPS (Needs Fixing)
% ===========================================================================
% Check \tilde{\bm{x}}(t) convergence time
% Thrseshold = 0.0001;
% idxBase = find(est_error_base < Threshold, 1, 'first');
% t_exp(idxBase)

% idxProposed = find(est_error_IM(:) < Threshold, 1, 'first');

% for simulated data ONLY
% ontime = sum(intermittentTimes(1:idxProposed))* tau

% for experiment data ONLY
% ontime = tau(1:idxProposed) .* intermittentTimes(1:idxProposed);
% ontime = sum(ontime)


% % Check \delta(t) convergence time
% Threshold = 0.0001;
% idxProposed = find(dist_error_IM(:) < Threshold, 1, 'first');
% trackingConv = idxProposed * tau

% ===========================================================================
%  H                        HELPER FUNCTIONS
% ===========================================================================

% Generates a boolean mask to simulate signal loss
%
% Returns   : If
% 1         : If we want there to be a bearing measurment
% 0         : If we want there to be intermission
function mask = generateIntermittentMask(N, minInt, maxInt, grace, tau, chance)
    
    mask = true(1, N);
    minInt = floor(minInt / tau);
    maxInt = floor(maxInt / tau);
    grace = floor(grace / tau) + 1;

    i = grace;
    while i < N
        i = i + 1;
        RNG = rand();
        if RNG < chance
            duration = minInt + floor(rand() * (maxInt - minInt));
            endIndex = i + duration;

            if endIndex > N
                endIndex = N;
            end

            mask(i:endIndex) = 0;

            i = endIndex + 1;
        end
    end
end
