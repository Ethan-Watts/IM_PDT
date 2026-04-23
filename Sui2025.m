function [y_data, est_error, dist_error] = main(N, Tau, init_x, init_x_hat, init_y, d_des, a1, a2, Tc1, Tc2, kw)
% This simulation covers the circumnavigation model proposed by Donglin Sui 2025
if size(Tau, 1) > 1
    isEXP = true;
else
    isEXP = false;
end

% Model Variables
global x_real
x_real = init_x;
x_hat = init_x_hat;
y = init_y;

% Pre-Defined Time Variables
P = zeros(2, 2);
q = [0; 0];
Xi = [0; 0];
dist = d_des;

% Simulation Variables
y_data = zeros(2, N);
est_error = zeros(1, N);
dist_error = zeros(1, N);

% Initialization
y_data(:, 1) = y;
est_error(1) = abs(norm(x_hat - x_real));
dist_error(1) = abs(norm(y - x_real)) - dist;

for i = 1:N
    angle = getBearing(y);
    if isEXP == false
        tau = Tau(1);
    else
        tau = Tau(i);
    end

    % Model Variables
    phi = [cos(angle); sin(angle)];
    phi_bar = [-sin(angle); cos(angle)];
    D_hat = norm(y - x_hat);
    
    % Model Equations
    [y_dot, D_tilde] = controlEquation(phi, phi_bar, a2, Tc2, D_hat, kw, dist);
    [x_hat_dot, Xi] = estimatorEquation(x_hat, P, q, a1, Tc1, i);
    P_dot = -P + (phi_bar*phi_bar');
    q_dot = -q + (phi_bar*phi_bar')*y;

    % Store Position Data
    y_data(:, i) = y;
    est_error(i) = abs(norm(x_hat - x_real));
    dist_error(i) = abs(norm(y - x_real))-dist;

     % Euler's Intgration
    y = y + tau * y_dot;
    x_hat = x_hat + tau * x_hat_dot;

    P = P + tau * P_dot;
    q = q + tau * q_dot;
end

end

function theta = getBearing(y)
    global x_real;
    theta = atan2(x_real(2) - y(2), x_real(1) - y(1));
end

function [u, d_tilde] = controlEquation(phi, phi_bar, a2, Tc2, d_hat, kw, dist)
    d_tilde = d_hat - dist;
    c = 1/(a2 * Tc2) * exp(abs(d_tilde)^(a2)) * sig(d_tilde, 1 - a2);
    u = c * phi + kw * phi_bar;
end


function [x_hat_dot, Xi] = estimatorEquation(x_hat, P, q, a1, Tc1, i)
    Xi = zeros(2,1);
    if rcond(P) > 0.00001
        Xi = inv(P) * (P * x_hat - q);
    end

    if norm(Xi) == 0
        psi = zeros(2,1);
    else
        psi = Xi ./ (norm(Xi)^a1);
    end

    x_hat_dot = -1/(a1 * Tc1) * exp(norm(Xi)^(a1)) * psi;
end


