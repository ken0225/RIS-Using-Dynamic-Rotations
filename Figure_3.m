% Author: Ke(Ken)WANG from Macao Polytechnic Institute Email:
% ke.wang@ipm.edu.mo, kewang0225@gmail.com Update infomation:
% v0.1(2020/03/01), v0.7(2022/01/02)


%% Clean All & Timer
close all;
clear;
tic;

%% System Parameters Initialization

global c0 fc speed M N; % Global parameters

% -------- Changeable parameters begin -------- You CAN change the
% parameters below to obtain different figures In this simulation we IGNORE
% the phase drifts

% kappa_t = 0; % Transceiver HWI (Transmitter side)
%
% kappa_r = kappa_t; % Transceiver HWI (Receiver side)
%
% a_HWI = pi/6; % RIS HWI, from 0 to pi/2

% Realization_HWI = 1; % The number of realizations, normally it is set to
% 1 but you can try another value say 10^3

M = 64; N = M; % IRS has M x N elements

bias_x_axis = 0; % the bias of IRS on the x-axis, normally it is set to 0

%  -------- Changeable parameters end --------

c0 = 299792458; % Light speed

fc= 2.4e9; % Carrier frequency is 2.4 GHz

disp(['Step 1: Parameter Initialization begin. In this setup, the IRS has ' num2str(M) ' x ' num2str(N) ' elements.']);

dx = (c0/fc)/(2*sqrt(pi)); dy = dx; % dxdy=\lambda^2/(4\pi), this means the element gain is 1.

h_IRS = 0; % Height of the IRS

h_V = 0; % Height of the vehicle

h_BS = 0; % Height of the BS

speed = 0.1; % Vehicle speed, note that 300km/h = 83.3333m/s

total_time = 200; % Total time for one vehicle pass

Pt = 0.1; % Transmit power is 0.1W = 30dBm

N0 = 2.5e-13; % N0 = -96dBm

% The coordinate of the vehicle, i.e., the receiver
x_Vehicle_start = -10;
y_Vehicle = 0+h_V;
z_Vehicle = 5;

% The coordinate of the Base Station, i.e., the transmitter
x_BS = -10;
y_BS = 0+h_BS;
z_BS = 5;

%% Compute the trajectory for one vehicle pass

disp(['Step 2: Compute the trajectory for one vehicle pass. The speed is ' num2str(speed) ' m/s.']);

p_BS = [x_BS, y_BS, z_BS];

p_IRS_0 = [0, h_IRS, 0];

p_vehicle_start = [x_Vehicle_start, y_Vehicle, z_Vehicle];

% Total trajectory = starting point + moving trajectory per second (we
% start from 1s rather than 0s)
p_vehicle_trajectory = [];

% The vehicle starts at p_vehicle_start, and travels at the speed during
% total_time period
for aa = 1 : total_time
    
    temp_p_vehicle_trajectory =  p_vehicle_start+function_vehicle_moving_xdir(speed, aa);
    
    p_vehicle_trajectory = [p_vehicle_trajectory; temp_p_vehicle_trajectory];
    
end

% 'A_0' is the gain for the direct path. It is a theoretical result.
[A_0] = function_A0(p_BS, p_vehicle_trajectory);

%% Compute the gain when the IRS is non-isotropic, after phase optimization, w/o HWI, w/o rotation
disp('Step 3.1: Compute the gain when the IRS is non-isotropic, after phase optimization, w/o HWI, w/o rotation.');

% 1/0 stands for isotropic IRS/practical IRS
isotropic = 0;

% Locate the positions for M and N
[centers_IRS] = function_centers_IRS(M, N, dx, dy);

% 'h_IRS' is the height of IRS
centers_IRS(:, 2) = centers_IRS(:, 2) + h_IRS;

% the bias of IRS on the x-axis
centers_IRS(:, 1) = centers_IRS(:, 1) + bias_x_axis;

% 'A_mn_theoretical' is the gain for the IRS path, and it is a theoretical
% result.
[A_mn,~] = function_Amn(p_BS, centers_IRS, p_vehicle_trajectory, total_time, dx, dy, isotropic);

% Total gain of theoretical result
A_mn_plus_A_0_wo_ratation = A_mn.'+A_0;

% Delay spread calculation
[tau_0_wo_rotation, tau_mn_wo_rotation] = function_time_delay(centers_IRS, p_BS, p_vehicle_trajectory);

phi_optimal_wo_rotation = 2*pi*(fc*(tau_0_wo_rotation-tau_mn_wo_rotation) + ceil(-(fc * (tau_0_wo_rotation-tau_mn_wo_rotation))));

Td_wo_rotation = max(tau_mn_wo_rotation+(phi_optimal_wo_rotation./(2*pi*fc))) - tau_0_wo_rotation;

%% Compute the gain when the IRS is non-isotropic, after phase optimization, w/o HWI, rotate 45 degree
disp('Step 3.2: Compute the gain when the IRS is non-isotropic, after phase optimization, w/o HWI, always towards the BS (i.e., rotate fixed degree).');

% 1/0 stands for isotropic IRS/practical IRS
isotropic = 0;

rotate_angle = - function_theta(p_BS, p_IRS_0);

%disp(rotate_angle*180/pi);

centers_IRS_rotated_initial = centers_IRS * function_rotate_IRS(rotate_angle);

% 'A_mn_theoretical' is the gain for the IRS path, and it is a theoretical
% result.
[A_mn_rotated,~] = function_Amn(p_BS, centers_IRS_rotated_initial, p_vehicle_trajectory, total_time, dx, dy, isotropic);

% Total gain of theoretical result
A_mn_plus_A_0_w_fixed_rotation = A_mn_rotated.'+A_0;

% Delay spread calculation
[tau_0_w_fixed_rotation, tau_mn_w_fixed_rotation] = function_time_delay(centers_IRS_rotated_initial, p_BS, p_vehicle_trajectory);

phi_optimal_w_fixed_rotation = 2*pi*(fc*(tau_0_w_fixed_rotation-tau_mn_w_fixed_rotation) +...
    ceil(-(fc * (tau_0_w_fixed_rotation-tau_mn_w_fixed_rotation))));

Td_w_fixed_rotation = max(tau_mn_w_fixed_rotation+(phi_optimal_w_fixed_rotation./(2*pi*fc))) - tau_0_w_fixed_rotation;

%% Compute the Gain when the IRS is Isotropic

disp('Step 6: Compute the gain when the IRS is isotropic, i.e., the upper bound.');

% 1/0 stands for isotropic/non-isotropic
isotropic = 1;

% 'A_mn_iso' is the gain when the IRS is isotropic, i.e., the upper bound.
[A_mn_iso,~] = function_Amn(p_BS, centers_IRS, p_vehicle_trajectory, total_time, dx, dy, isotropic);

% 'A_mn_iso' is total gain, it is a theoretical result, which means it
% cannot be achieved
A_mn_total_iso = A_mn_iso.' + A_0;

%% Dynamic Rotating

disp('Step 8: Compute the gain for the danamic rotation without error');

[temp_rotate_theta_opt]=function_theta(p_vehicle_trajectory, p_IRS_0);

rotate_vector_part1 = pi/4-temp_rotate_theta_opt(1:total_time/2);

rotate_vector_part2=temp_rotate_theta_opt(total_time/2+1:total_time)+pi/4;

rotate_vector = [rotate_vector_part1,rotate_vector_part2]/2;

for bb = 1 : length(rotate_vector)
    
    rotate_matrix_tensor(:,:,bb) = function_rotate_IRS(rotate_vector(bb)); % size(rotate_matrix) = 3 x 3 x total_time
    
end

for cc = 1 : length(rotate_vector) % length(rotate_vector) = total_time
    
    centers_IRS_rotated_tensor(:,:,cc) = centers_IRS_rotated_initial * rotate_matrix_tensor(:,:,cc);
    
end

isotropic = 0;

for dd = 1 : length(rotate_vector) % length(rotate_vector) = total_time
    
    [A_mn_w_dynamic_rotation(dd), A_mn_matrixw_dynamic_rotation(dd,:)]=function_Amn(p_BS, centers_IRS_rotated_tensor(:,:,dd), p_vehicle_trajectory(dd,:), 1, dx, dy, isotropic);
end

A_mn_plus_A_0_w_dynamic_rotation = A_mn_w_dynamic_rotation+A_0;

inst_received_power_IRS_w_dynamic_rotation = Pt * (abs(A_mn_plus_A_0_w_dynamic_rotation)) .^ 2;
inst_SNR_IRS_w_dynamic_rotation = inst_received_power_IRS_w_dynamic_rotation ./ (N0);
SE_IRS_w_dynamic_rotation = log2(1 + inst_SNR_IRS_w_dynamic_rotation);

%% Analytical result
disp('Step 9: Compute the SE Approximation');

A_star = [];
Q = [];

% Compute the A*
for t = 1 : total_time
    
    temp_A_star=function_Astar(A_mn_matrixw_dynamic_rotation(t,:));
    A_star = [A_star; temp_A_star];
    
end

% Compute the Q
for t = 1 : total_time
    
    temp_Q = function_Q(A_0(t), A_star(t), A_mn_matrixw_dynamic_rotation(t,:));
    Q(t) = temp_Q;
    
end

Q = Pt*Q;

% Compute the SNR approximation
SNR_approximation = (Q ./ N0)';

% Compute the SE approximation
SE_approximation = log2(1+SNR_approximation);


%% Delay spread calculation

for ee = 1 : length(rotate_vector) % length(rotate_vector) = total_time
    
[temp_tau_0_w_dynamic_rotation, temp_tau_mn_w_dynamic_rotation] = ...
function_time_delay(centers_IRS_rotated_tensor(:,:,ee), p_BS, p_vehicle_trajectory(ee,:)); 

tau_0_w_dynamic_rotation(ee) = temp_tau_0_w_dynamic_rotation;
tau_mn_w_dynamic_rotation(:,ee) = temp_tau_mn_w_dynamic_rotation;
    
end 


phi_optimal_w_dynamic_rotation = 2*pi*(fc*(tau_0_w_dynamic_rotation-tau_mn_w_dynamic_rotation) +...
    ceil(-(fc * (tau_0_w_dynamic_rotation-tau_mn_w_dynamic_rotation))));

Td_w_dynamic_rotation = max(tau_mn_w_dynamic_rotation+(phi_optimal_w_dynamic_rotation./(2*pi*fc))) - tau_0_w_dynamic_rotation;

%% Figure 1

close all

inst_received_power_no_IRS = Pt * (abs(A_0)) .^ 2;
inst_SNR_no_IRS = inst_received_power_no_IRS ./ (N0);
SE_no_IRS = log2(1 + inst_SNR_no_IRS);

inst_received_power_IRS_wo_ratation = Pt * (abs(A_mn_plus_A_0_wo_ratation)) .^ 2;
inst_SNR_IRS_wo_ratation = inst_received_power_IRS_wo_ratation ./ (N0);
SE_IRS_wo_rotation = log2(1 + inst_SNR_IRS_wo_ratation);

inst_received_power_IRS_w_fixed_rotation = Pt * (abs(A_mn_plus_A_0_w_fixed_rotation)) .^ 2;
inst_SNR_IRS_w_fixed_rotation = inst_received_power_IRS_w_fixed_rotation ./ (N0);
SE_IRS_w_fixed_rotation = log2(1 + inst_SNR_IRS_w_fixed_rotation);

inst_received_power_isotropic_IRS = Pt * (abs(A_mn_total_iso)) .^ 2;
inst_SNR_isotropic_IRS = inst_received_power_isotropic_IRS ./ ((N0));
SE_isotropic_IRS = log2(1 + inst_SNR_isotropic_IRS);

figure(1); hold on; box on; grid on;

moving_distance = linspace(1,total_time,total_time) .* speed;

p1 = plot(moving_distance, SE_no_IRS, 'k:', 'LineWidth', 2);
p2 = plot(moving_distance, SE_IRS_wo_rotation, 'r--', 'LineWidth', 2);
p3 = plot(moving_distance, SE_IRS_w_fixed_rotation, 'g-', 'LineWidth', 2);
p4  = plot(moving_distance, SE_IRS_w_dynamic_rotation, 'b-.', 'LineWidth', 2);
p5 = plot(moving_distance, SE_isotropic_IRS, 'k-', 'LineWidth', 3);
p6 = plot([2,4,6,8,10,12], [24.86,24.22,24.24,24.36,24.22,23.6], '+', 'LineWidth', 3, 'MarkerSize', 12);

xlabel('Moving Distance (m)','Interpreter','LaTex');
ylabel('Spectral Efficiency (bit/s/Hz)','Interpreter','LaTex');

legend([p1(1),p2(1),p3(1),p4(1),p5(1),p6(1)],...
    'w/o RIS','w/ RIS w/o ratation','w/ RIS w/ $45^\circ$ ratation',...
    'w/ RIS w/ dynamic rotation','w/ Isotropic RIS','Analytical SE in Eq. (20)','Interpreter','LaTex', 'Location', 'SouthWest');
axis([1, 12.5, 18, 26]);

%% Figure 2
figure(2); hold on; box on; grid on;

moving_distance = linspace(1,total_time,total_time) .* speed;

p1 = plot(moving_distance, Td_wo_rotation, 'k-', 'LineWidth', 2);
p2 = plot(moving_distance, Td_w_dynamic_rotation, 'r--', 'LineWidth', 2);

xlabel('Moving Distance (m)','Interpreter','LaTex');
ylabel('Delay spread (s)','Interpreter','LaTex');

legend([p1(1),p2(1)],...
    'w/ RIS w/o ratation',...
    'w/ RIS w/ dynamic rotation','Interpreter','LaTex');
axis([1, 12.5, 1e-8, 8e-8]);

toc;


