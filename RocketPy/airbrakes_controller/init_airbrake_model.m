clear; clc;

%% Simulation timing
Ts_plant = 0.001;      % Plant integration nominal step [s]
Ts_ctrl  = 0.05;       % Controller update rate [s], 20 Hz
Ts_baro  = 0.02;       % Barometer update rate [s], 50 Hz
Ts_imu   = 0.005;      % IMU update rate [s], 200 Hz
Ts_gps   = 0.10;       % Optional GPS update rate [s], 10 Hz

t_final = 60;          % Simulation stop time [s]

%% Rocket geometry / mass
% If this is the same RocketPy model you have used before:
R_rocket = 0.078359;                     % Rocket radius [m]
A_ref = pi * R_rocket^2;                 % Reference area [m^2]
m0 = 23.243;                             % Initial mass [kg]

% If using another rocket, replace these directly from OpenRocket/RocketPy.

%% Gravity and atmosphere constants
g0 = 9.80665;                            % Gravity [m/s^2]
rho0 = 1.225;                            % Sea-level air density [kg/m^3]
T0 = 288.15;                             % Sea-level standard temperature [K]
p0 = 101325;                             % Sea-level pressure [Pa]
L = 0.0065;                              % Troposphere lapse rate [K/m]
R_air = 287.05;                          % Specific gas constant for air [J/(kg*K)]
gamma_air = 1.4;                         % Ratio of specific heats
h_launch = 0;                            % Altitude above launch site [m]

%% Target
h_apogee_target = 3000;                  % Desired apogee AGL [m], CHANGE THIS

%% Motor data
% Replace with actual thrust curve from RocketPy/OpenRocket motor file.
% Use seconds and Newtons.
motor_time = [0 0.05 0.10 0.50 1.00 1.50 2.00 2.50 3.00 3.20 3.30];
motor_thrust = [0 1200 2500 3300 3400 3200 2800 1900 700 100 0];

burnout_time = motor_time(end);

% Replace with actual mass curve if you have it.
% If not, approximate propellant burn linearly.
m_dry = 17.5;                            % Dry mass [kg], placeholder
mass_time = motor_time;
mass_curve = linspace(m0, m_dry, numel(mass_time));

%% Body drag data from OpenRocket/RocketPy
% Replace these with your exported Cd vs Mach data.
Mach_bp = [0 0.1 0.3 0.5 0.7 0.85 0.95 1.05 1.2 1.5 2.0];
Cd_body_bp = [0.45 0.45 0.43 0.42 0.44 0.50 0.65 0.75 0.62 0.52 0.48];

%% Airbrake delta-Cd table
% Deployment from 0 to 1.
deploy_bp = [0 0.25 0.50 0.75 1.00];

% Mach breakpoints should match or cover Mach_bp.
Mach_ab_bp = Mach_bp;

% Rows = deployment, columns = Mach.
% Placeholder values. Replace with CFD/RocketPy/OpenRocket airbrake table.
DeltaCd_ab_table = [
    0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00
    0.04 0.04 0.05 0.06 0.07 0.08 0.10 0.11 0.10 0.09 0.08
    0.09 0.09 0.10 0.12 0.14 0.16 0.19 0.21 0.19 0.17 0.15
    0.15 0.15 0.17 0.20 0.23 0.26 0.31 0.34 0.31 0.27 0.24
    0.22 0.22 0.25 0.29 0.33 0.38 0.45 0.50 0.45 0.40 0.35
];

%% Airbrake actuator / servo
u_min = 0;
u_max = 1;

servo_tau = 0.05;                        % First-order servo time constant [s]
servo_delay = 0.03;                      % Command/servo delay [s]
deploy_rate_up = 6.67;                   % 0 to 1 in about 0.15 s [1/s]
deploy_rate_down = 6.67;                 % 1 to 0 in about 0.15 s [1/s]

servo_quant_step = 1/255;                % Optional PWM/command quantization

%% Controller tuning
deadband_m = 5;                          % Ignore apogee error within +/- 5 m
Kp_apogee = 1/300;                       % Full deploy for about 300 m overshoot
Ki_apogee = 0;                           % Start at zero
Kd_apogee = 0;                           % Usually not needed initially

mpc_candidate_grid = 0:0.05:1;           % Candidate deployment values
mpc_pred_dt = 0.05;                      % Predictor integration step [s]
mpc_max_pred_time = 60;                  % Max forward prediction time [s]
mpc_du_weight = 5000;                    % Penalizes aggressive deployment changes

%% Sensor noise
% Barometer altitude model
baro_sigma_h = 0.5;                      % Altitude noise std dev [m]
baro_bias_initial = 1.0;                 % Initial barometer bias [m]
baro_bias_rw_sigma = 0.02;               % Bias random walk [m/sqrt(s)]
baro_delay = 0.04;                       % Barometer processing delay [s]

% IMU accelerometer model
imu_sigma_a = 0.15;                      % Accel noise std dev [m/s^2]
imu_bias_initial = 0.05;                 % Initial accel bias [m/s^2]
imu_bias_rw_sigma = 0.01;                % Bias random walk [m/s^2/sqrt(s)]
imu_delay = 0.005;                       % IMU delay [s]
imu_accel_range_g = 16;                  % +/-16 g range
imu_quant_step = (2*imu_accel_range_g*g0)/(2^16);

% Optional GPS
gps_sigma_h = 3.0;                       % GPS altitude noise std dev [m]
gps_delay = 0.20;                        % GPS delay [s]

%% Observer tuning
sigma_baro_for_filter = baro_sigma_h;
sigma_accel_for_filter = imu_sigma_a;
sigma_accel_bias_rw_filter = 0.03;

P0_obs = diag([10^2, 10^2, 0.5^2]);      % Initial covariance for [h, v, accel_bias]
Q_obs = diag([0.01^2, 0.1^2, sigma_accel_bias_rw_filter^2]);
R_obs = sigma_baro_for_filter^2;