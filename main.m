%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Zengjie Zhang
% Affliation: The University of British Columbia
% Email: zengjie.zhang@ubc.ca

clear;
clc;

Ts = 10e-5;
Tn = 10;
t_scale = 0:Ts:Tn;
trajectory_gain = [0.5;0.8;0.2];
disturbance_gain = [0.15;0.18;0.12];

x_des = generate_trajectory(t_scale, trajectory_gain);
d_sin = disturbance_sin(t_scale, disturbance_gain);
d_sqrt = disturbance_sqrt(t_scale, disturbance_gain);
d_tri = disturbance_trg(t_scale, disturbance_gain);

% Run sinusoidal disturbance simulation
[x1, x2, u, s_ismo, x1_est_ismo, x2_est_ismo, d_sin_est_ismo, s_smo, x1_est_smo, x2_est_smo, d_sin_est_smo] = sim_manipulator(Ts, Tn, d_sin, x_des);

% Run triangle-shape disturbance simulation
[~, ~, ~, ~, ~, ~, d_tri_est_ismo, ~, ~, ~] = sim_manipulator(Ts, Tn, d_tri, x_des);

% Run square-shape disturbance simulation
[~, ~, ~, ~, ~, ~, d_sqrt_est_ismo, ~, ~, ~] = sim_manipulator(Ts, Tn, d_sqrt, x_des);


% Figure 1

draw_figure(t_scale, x1, [0,10,0,2], '{\boldmath{$x$}}$_1(t)$');
draw_figure(t_scale, x2, [0,10,-1.5,1.5], '{\boldmath{$x$}}$_2(t)$');
draw_figure(t_scale, u, [0,10,-2,3], '{\boldmath{$u$}}$(t)$');
draw_figure(t_scale, d_sin, [0,10,0,0.2], '{\boldmath{$d$}}$(t)$');


% Figure 2
draw_figure(t_scale, s_ismo, [0,10,-1,0.4], '{\boldmath{$s$}}$(t)$');
draw_figure(t_scale, x1_est_ismo, [0,10,0,2], '{\boldmath{$\hat{x}$}}$_1(t)$');
draw_figure(t_scale, x2_est_ismo, [0,10,-1.5,1.5], '{\boldmath{$\hat{x}$}}$_2(t)$');
draw_figure(t_scale, d_sin_est_ismo, [0,10,0,0.2], '{\boldmath{$\hat{d}$}}$(t)$');
draw_figure(t_scale, x1-x1_est_ismo, [0,10,-8e-11,8e-11+1e-16], '{\boldmath{$\tilde{x}$}}$_1(t)$');
draw_figure(t_scale, x2-x2_est_ismo, [0,10,-8e-3,8e-3], '{\boldmath{$\tilde{x}$}}$_2(t)$');
draw_figure(t_scale, d_sin-d_sin_est_ismo, [0,10,-5e-3,5e-3], '{\boldmath{$\tilde{d}$}}$(t)$');


% Figure 3
draw_figure(t_scale, s_smo, [0,10,-1,0.4], '{\boldmath{$s''$}}$\!(t)$');
draw_figure(t_scale, x1_est_smo, [0,10,0,2], '{\boldmath{$\hat{x}''$}}$\!_1(t)$')
draw_figure(t_scale, x2_est_smo, [0,10,-1.5,1.5], '{\boldmath{$\hat{x}''$}}$\!_2(t)$')
draw_figure(t_scale, d_sin_est_smo, [0,10,0,0.2], '{\boldmath{$\hat{d}''$}}$\!(t)$')
draw_figure(t_scale, x1-x1_est_smo, [0,10,-4e-3,4e-3], '{\boldmath{$\tilde{x}''$}}$\!_1(t)$')
draw_figure(t_scale, x2-x2_est_smo, [0,10,-8e-3,8e-3], '{\boldmath{$\tilde{x}''$}}$\!_2(t)$')
draw_figure(t_scale, d_sin-d_sin_est_smo, [0,10,-0.015,0.015], '{\boldmath{$\tilde{d}''$}}$\!(t)$')


% Figure 4
draw_figure(t_scale, d_tri, [0,10,0,0.2], '{\boldmath{$d$}}$_{\mathrm{trg}}(t)$')
draw_figure(t_scale, d_tri_est_ismo, [0,10,0,0.2], '{\boldmath{$\hat{d}$}}$_{\mathrm{trg}}(t)$')
draw_figure(t_scale, d_tri-d_tri_est_ismo, [0,10,-4e-3,4e-3], '{\boldmath{$\tilde{d}$}}$_{\mathrm{trg}}(t)$')
draw_figure(t_scale, d_sqrt, [0,10,0,0.2], '{\boldmath{$d$}}$_{\mathrm{sqr}}(t)$')
draw_figure(t_scale, d_sqrt_est_ismo, [0,10,0,0.2], '{\boldmath{$\hat{d}$}}$_{\mathrm{sqrt}}(t)$')
draw_figure(t_scale, d_sqrt-d_sqrt_est_ismo, [0,10,-0.2,0.2], '{\boldmath{$\tilde{d}$}}$_{\mathrm{sqrt}}(t)$')
