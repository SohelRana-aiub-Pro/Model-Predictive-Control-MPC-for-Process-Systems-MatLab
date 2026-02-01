%% PURE OCTAVE IMPLEMENTATION OF MPC (NO CONTROL PACKAGE)
% First-order plus dead-time (FOPDT) process
% Model Predictive Control with greedy search

clc; clear; close all;

%% Step 1: Define Process Parameters
K = 2;      % Process gain
T = 10;     % Time constant
L = 3;      % Dead time (in samples)
Ts = 1;     % Sampling time
N = 100;    % Simulation length

% Discrete-time approximation (Euler)
a = exp(-Ts/T);          % decay factor
b = K*(1-a);             % input gain

%% Step 2: MPC Parameters
PredictionHorizon = 20;
ControlHorizon = 5;

lambda = 0.1;   % weight on input moves
Q = 1;          % weight on output error

u_min = 0; u_max = 100;   % input constraints

%% Step 3: Initialize Variables
y = zeros(N,1);       % process output
u = zeros(N,1);       % control input
r = ones(N,1)*50;     % setpoint = 50
disturbance = zeros(N,1);
disturbance(40:60) = 10; % synthetic disturbance

% Dead-time buffer
u(1) = 10;                     % initial kick
u_buffer = ones(L,1)*u(1);     % initialize buffer with non-zero input

%% Step 4: MPC Simulation Loop
for k = 2:N
    
    % --- Process model update ---
    u_delayed = u_buffer(end);
    u_buffer = [u(k-1); u_buffer(1:end-1)];
    
    y(k) = a*y(k-1) + b*u_delayed + disturbance(k);
    
    % --- MPC optimization ---
    candidates = linspace(u_min,u_max,20); % test 20 possible inputs
    bestJ = Inf; bestU = u(k-1);
    
    for cand = candidates
        y_temp = y(k);
        u_buf_temp = [cand; u_buffer(1:end-1)];
        u_temp = u_buf_temp(end);
        y_pred_test = zeros(PredictionHorizon,1);
        
        for j = 1:PredictionHorizon
            y_temp = a*y_temp + b*u_temp;
            y_pred_test(j) = y_temp;
        end
        
        % Match vector lengths to avoid dimension mismatch
        ref_segment = r(k:min(N,k+PredictionHorizon-1));
        len = length(ref_segment);
        J = sum(Q*(ref_segment - y_pred_test(1:len)).^2) ...
            + lambda*(cand - u(k-1))^2;
        
        if J < bestJ
            bestJ = J;
            bestU = cand;
        end
    end
    
    % Apply best control move
    u(k) = bestU;
end

%% Step 5: Plot Results
figure;
subplot(2,1,1);
plot(1:N,y,'b','LineWidth',1.5); hold on;
plot(1:N,r,'r--','LineWidth',1.5);
xlabel('Time step'); ylabel('Output');
legend('Output','Setpoint');
title('MPC Control of Process System (Pure Octave)');
grid on;



subplot(2,1,2);
stairs(1:N,u,'k','LineWidth',1.5);
xlabel('Time step'); ylabel('Control Input');
title('MPC Control Input');
grid on;