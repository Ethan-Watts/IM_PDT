function [y_data, est_error, dist_error] = main(N, Tau, init_x, init_x_hat, init_y, d_des, intermittentTimes, a1, a2, Tc1, Tc2, kw)
% This simulation covers the MODIFIED circumnavigation model proposed by Donglin Sui 2025
if size(Tau, 1) > 1
    isEXP = true;
else
    isEXP = false;
end

% Initialise BoTLC Variables
global x_real
x_real = init_x;
x_hat = init_x_hat;
y = init_y;
d_star = d_des;

% Initialise Pre-Defined Time Variables
P = zeros(2, 2);
q = [0; 0];
Xi = [0; 0];
XiThresh = 0.000000000001;  % Denoted as \mathcal{E} in the Thesis

% Simulation Variables
y_data = zeros(2, N);
est_error = zeros(1, N);
dist_error = zeros(1, N);

% Initialization
y_data(:, 1) = y;
est_error(1) = abs(norm(x_hat - x_real));
dist_error(1) = abs(norm(y - x_real)) - d_star;

for i = 1:N
    angle_x = getBearing(y, intermittentTimes(i));
    if isEXP == false
        tau = Tau(1);
    else
        tau = Tau(i);
    end

    if isnan(angle_x)
        % Unavailable
        P_dot = 0;
        q_dot = 0;

        if rcond(P) > XiThresh
            Xi =  x_hat - P\q;
        end
        x_hat_dot = 0;
    else
        % Available
        phi = [cos(angle_x); sin(angle_x)];
        phi_bar = [-sin(angle_x); cos(angle_x)];

        P_dot = -P + (phi_bar*phi_bar');
        q_dot = -q + (phi_bar*phi_bar')*y;

        if rcond(P) > XiThresh
            Xi =  x_hat - P\q;
        end
        
        if norm(Xi) == 0
            psi = zeros(2,1);
        else
            psi = Xi ./ (norm(Xi)^a1);
        end
        x_hat_dot = -1/(a1 * Tc1) * exp(norm(Xi)^(a1)) * psi;
    end
    
    % Virtual Point
    O = x_hat - Xi;
    angle_O = atan2(O(2) - y(2), O(1) - y(1));

    phiO = [cos(angle_O); sin(angle_O)];
    phiO_bar = [-sin(angle_O); cos(angle_O)];
    d_O = norm(O - y);
    
    % Circumnavigation Controller
    if  rcond(P) > XiThresh
        y_dot = controlEquation(phiO, phiO_bar, a2, Tc2, d_O, kw, d_star);
    else
        y_dot = kw * phiO_bar;
    end

    % Store Runtime Data
    y_data(:, i) = y;
    est_error(i) = abs(norm(x_hat - x_real));
    dist_error(i) = abs(norm(y - x_real))-d_star;

    % Euler's Integration
    y = y + tau * y_dot;
    x_hat = x_hat + tau * x_hat_dot;

    P = P + tau * P_dot;
    q = q + tau * q_dot;

end
end


% Returns the REAL bearing to the target, or returns nan during intermittent times
%   This works in conjunction with the Intermittent boolean mask
% 
% Returns       : If
% bearing angle : If the mask is 1
% nan           : If the mask is 0
function theta = getBearing(y, IM)
    if IM == 1
        global x_real;
        theta = atan2(x_real(2) - y(2), x_real(1) - y(1));
    else
        theta = nan;
    end
end

% Circumnavigation Controller Calculation
function y_dot = controlEquation(phi, phi_bar, a2, Tc2, d_O, kw, dist)
    d_tilde = d_O - dist;
    c = 1/(a2 * Tc2) * exp(abs(d_tilde)^(a2)) * sig(d_tilde, 1 - a2);
    y_dot = c * phi + kw * phi_bar;
end

