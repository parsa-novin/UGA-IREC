%% airbrake_closed_loop_sim.m
%
% Theoretical closed-loop airbrake simulation for MAGS rocket.
%
% PHASE 1 — Open-loop (no airbrakes): finds natural apogee.
% PHASE 2 — Closed-loop (PID + EKF): targets (natural_apogee - 100 m).
%
% Physics  : 1-D vertical flight, RK4, ISA atmosphere.
% Estimator: EKF on [altitude, velocity] — IMU drives predict, baro corrects.
% Controller: PID on (predicted_apogee - Z_TARGET).
%             Apogee predicted via energy-balance with current drag state.
%
% Motor: m2050x_corrected.eng  (exact thrust curve and masses)
%
% Subplots (3×3):
%   [1] Altitude  (open-loop | closed-loop + EKF ±2σ)
%   [2] Velocity  (open-loop | closed-loop + EKF ±2σ)
%   [3] Drag Force
%   [4] Airbrake Deployment Angle
%   [5] Mach Number
%   [6] Motor Thrust Curve  (m2050x_corrected.eng)
%   [7] Predicted Apogee vs Target
%   [8] PID Error Signal & Deployment Level
%   [9] Net Acceleration (m/s² and g)

clear; clc; close all;
rng(42);   % reproducible noise

%% ================================================================
%  ROCKET PARAMETERS  (match mags_test_launch.py exactly)
%% ================================================================
R_body  = 0.078359;           % body radius [m]
A_ref   = pi * R_body^2;     % reference area [m²]  ≈ 0.019293 m²
m_dry   = 18.6;              % dry rocket mass (no motor) [kg]

% m2050x_corrected.eng header:
%   M2050X 75 700 - 1.990 4.281 AT
%   propellant_mass = 1.990 kg,  total_motor_mass = 4.281 kg
m_prop  = 1.990;             % propellant mass [kg]
m_case  = 4.281 - m_prop;   % motor hardware (case + nozzle) = 2.291 kg
m0      = m_dry + m_case + m_prop;  % initial total mass = 22.881 kg

% Exact thrust-time points from m2050x_corrected.eng
T_eng = [0.000,    0.000;
         0.028,  417.223;
         0.067, 2562.075;
         0.151, 2409.045;
         0.261, 2391.860;
         0.372, 2297.596;
         0.557, 2265.367;
         0.779, 2242.435;
         0.965, 2220.179;
         1.150, 2188.683;
         1.372, 2145.748;
         1.816, 2059.655;
         2.149, 2027.933;
         2.225, 1100.000;
         2.350,    0.000];

t_burn    = 2.350;   % burnout time [s]
AB_MAX    = 80.0;    % max airbrake deployment angle [°]

thrust_fn = @(t) interp1(T_eng(:,1), T_eng(:,2), ...
                          min(max(t,0), t_burn), 'linear', 0);
mass_fn   = @(t) m0 - m_prop * min(t, t_burn) / t_burn;

%% ================================================================
%  ATMOSPHERE MODEL  (ISA troposphere)
%% ================================================================
T0  = 288.15;  L  = 0.0065;  P0 = 101325;
Ra  = 287.05;  ga = 1.4;     g0 = 9.80665;

T_fn   = @(z) T0 - L * max(z, 0);
rho_fn = @(z) (P0/(Ra*T0)) * max(1 - L*max(z,0)/T0, 1e-6).^(g0/(Ra*L) - 1);
a_fn   = @(z) sqrt(ga * Ra * T_fn(z));   % speed of sound [m/s]

%% ================================================================
%  AERODYNAMIC MODEL
%% ================================================================
% Base Cd vs Mach — representative for a 6-inch HPR airframe
Mach_tbl = [0.00, 0.20, 0.40, 0.60, 0.80, 0.90, 1.00, 1.10, 1.20, 1.50, 2.00];
Cd_tbl   = [0.50, 0.50, 0.51, 0.52, 0.56, 0.63, 0.72, 0.68, 0.62, 0.54, 0.47];
Cd_base  = @(M) interp1(Mach_tbl, Cd_tbl, min(max(M,0), 2.0), 'pchip');

% Airbrake ΔCd — mirrors AirbrakeModel.drag_coefficient_curve():
%   extra_area [m²] = 3982.98097 × sin(angle_rad) / 1e6
%   effective_Cd    = delta_Cd × (1 + extra_area / A_ref)
AB_AREA_K  = 3982.98097;   % from AirbrakeModel._AREA_CONST_MM2
AB_CD_PEAK = 0.38;         % ΔCd at full deployment, low Mach
Cd_AB = @(lev, M) ...
    max(0, lev * AB_CD_PEAK * max(0, 1 - 0.30*M)) .* ...
    (1 + (AB_AREA_K * sin(deg2rad(lev * AB_MAX)) / 1e6) / A_ref);

% Total drag magnitude [N]  (sign applied in EOM via sign(v))
drag_fn = @(z, v, lev) ...
    0.5 * rho_fn(max(z,0)) * v.^2 .* ...
    (Cd_base(abs(v)./a_fn(max(z,0))) + Cd_AB(lev, abs(v)./a_fn(max(z,0)))) * A_ref;

%% ================================================================
%  SIMULATION PARAMETERS
%% ================================================================
dt    = 0.01;    % timestep [s]
T_END = 70.0;    % max sim time [s]
t_vec = 0:dt:T_END;
N     = length(t_vec);

%% ================================================================
%  PHASE 1 — OPEN-LOOP (no airbrakes)
%  Find natural apogee; Z_TARGET = apogee - 100 m
%% ================================================================
fprintf('Running Phase 1: open-loop (no airbrakes)...\n');

x_ol      = [0.0; 0.0];
z_ol      = zeros(1, N);
v_ol      = zeros(1, N);
i_ol_end  = N;

for i = 1:N
    z_ol(i) = x_ol(1);
    v_ol(i) = x_ol(2);

    % Stop when rocket returns to ground after launch
    if x_ol(1) <= 0 && x_ol(2) < 0 && t_vec(i) > 5.0
        i_ol_end = i;
        break;
    end

    if i < N
        k1 = eom(x_ol,          t_vec(i),        0, thrust_fn, mass_fn, drag_fn, g0);
        k2 = eom(x_ol+dt/2*k1,  t_vec(i)+dt/2,  0, thrust_fn, mass_fn, drag_fn, g0);
        k3 = eom(x_ol+dt/2*k2,  t_vec(i)+dt/2,  0, thrust_fn, mass_fn, drag_fn, g0);
        k4 = eom(x_ol+dt*k3,    t_vec(i)+dt,    0, thrust_fn, mass_fn, drag_fn, g0);
        x_ol = x_ol + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        x_ol(1) = max(x_ol(1), 0.0);
    end
end

apogee_ol = max(z_ol(1:i_ol_end));
Z_TARGET  = apogee_ol - 100.0;

fprintf('  Natural apogee (no airbrakes): %.1f m AGL  (%.0f ft)\n', ...
        apogee_ol, apogee_ol*3.28084);
fprintf('  PID target (apogee - 100 m) : %.1f m AGL  (%.0f ft)\n\n', ...
        Z_TARGET, Z_TARGET*3.28084);

%% ================================================================
%  EKF PARAMETERS
%  State  : x = [z; v]
%  Predict: x_{k+1} = A*x_k + [0; a_imu]*dt   (IMU as process input)
%  Update : barometer measures z with noise σ_baro
%% ================================================================
sig_baro = 2.0;    % barometer 1-σ [m]
sig_imu  = 1.2;    % IMU 1-σ [m/s²]

Q_ekf  = diag([0.5*(sig_imu*dt^2)^2, (sig_imu*dt)^2]);
R_baro = sig_baro^2;
H_bar  = [1, 0];

%% ================================================================
%  PID PARAMETERS
%  Error  : e = predicted_apogee − Z_TARGET  [m]
%  Output : deployment_level ∈ [0, 1]
%  Arms   : 0.5 s after burnout while ascending
%% ================================================================
Kp = 5.0e-4;   % [level / m]
Ki = 5.0e-6;   % [level / (m·s)]
Kd = 2.0e-3;   % [level / (m/s)]

%% ================================================================
%  PHASE 2 — CLOSED-LOOP (PID + EKF)
%% ================================================================
fprintf('Running Phase 2: closed-loop (PID + EKF)...\n');

% Pre-allocate logs
z_true   = zeros(1,N);  v_true   = zeros(1,N);
z_est    = zeros(1,N);  v_est    = zeros(1,N);
Pz_var   = zeros(1,N);  Pv_var   = zeros(1,N);
lev_log  = zeros(1,N);  ang_log  = zeros(1,N);
Fd_log   = zeros(1,N);  Ft_log   = zeros(1,N);
Mach_log = zeros(1,N);  mass_log = zeros(1,N);
zpred_log= zeros(1,N);  pid_log  = zeros(1,N);
accel_log= zeros(1,N);  pid_err  = zeros(1,N);

% State and controller init
x_cl       = [0.0; 0.0];
x_ekf      = [0.0; 0.0];
P_ekf      = diag([4.0, 0.25]);
I_pid      = 0.0;
err_prev   = 0.0;
ctrl_armed = false;
apo_done   = false;
i_cl_end   = N;

for i = 1:N
    t   = t_vec(i);
    lev = lev_log(i);

    %% True state ------------------------------------------------
    z = x_cl(1);  v = x_cl(2);
    z_true(i) = z;  v_true(i) = v;

    %% Mass, thrust, aero ----------------------------------------
    m_curr = mass_fn(t);
    Ft     = thrust_fn(t);
    a_snd  = a_fn(max(z, 0));
    Mach   = abs(v) / a_snd;
    Fd     = drag_fn(max(z,0), v, lev);
    a_net  = (Ft - Fd*sign(v) - m_curr*g0) / m_curr;

    mass_log(i)  = m_curr;
    Ft_log(i)    = Ft;
    Fd_log(i)    = Fd;
    Mach_log(i)  = Mach;
    accel_log(i) = a_net;

    %% EKF predict -----------------------------------------------
    a_imu  = a_net + randn()*sig_imu;
    x_pred = [x_ekf(1) + x_ekf(2)*dt;
              x_ekf(2) + a_imu*dt];
    A_jac  = [1, dt; 0, 1];
    P_pred = A_jac * P_ekf * A_jac' + Q_ekf;

    %% EKF update (barometer) ------------------------------------
    z_baro = z + randn()*sig_baro;
    innov  = z_baro - H_bar * x_pred;
    S      = H_bar * P_pred * H_bar' + R_baro;
    K      = P_pred * H_bar' / S;
    x_ekf  = x_pred + K * innov;
    P_ekf  = (eye(2) - K*H_bar) * P_pred;
    x_ekf(1) = max(x_ekf(1), 0.0);

    z_est(i)  = x_ekf(1);
    v_est(i)  = x_ekf(2);
    Pz_var(i) = P_ekf(1,1);
    Pv_var(i) = P_ekf(2,2);

    %% Apogee predictor — numerical forward integration -----------
    % Integrates the coast EOM forward from EKF state until v≤0.
    % Holding lev constant gives a conservative (worst-case) estimate
    % of where the rocket will go if no further control action is taken.
    if x_ekf(2) > 0
        z_ap_pred = predict_apogee(x_ekf(1), x_ekf(2), lev, m_curr, drag_fn, g0);
    else
        z_ap_pred = x_ekf(1);
    end
    zpred_log(i) = z_ap_pred;

    %% PID controller --------------------------------------------
    % Arm once: after burnout + settling time, while still ascending.
    if ~ctrl_armed && ~apo_done && t > t_burn + 0.5 && x_ekf(2) > 2.0
        ctrl_armed = true;
    end
    % Disarm only after the controller has already been armed (prevents
    % v=0 at t=0 from immediately locking out the controller forever).
    if ctrl_armed && x_ekf(2) <= 0
        apo_done   = true;
        ctrl_armed = false;
    end

    new_lev = lev;

    if ctrl_armed
        e        = z_ap_pred - Z_TARGET;
        I_pid    = I_pid + e * dt;
        D_pid    = (e - err_prev) / dt;
        err_prev = e;
        raw      = Kp*e + Ki*I_pid + Kd*D_pid;
        pid_log(i) = raw;
        pid_err(i) = e;
        new_lev    = max(0.0, min(1.0, raw));
    end

    ang_log(i) = lev * AB_MAX;
    if i < N
        lev_log(i+1) = new_lev;
        ang_log(i+1) = new_lev * AB_MAX;
    end

    %% Stop at touchdown -----------------------------------------
    if z <= 0 && v < 0 && t > 5.0
        i_cl_end = i;
        lev_log(i+1:end) = 0;
        ang_log(i+1:end) = 0;
        break;
    end

    %% RK4 true dynamics -----------------------------------------
    if i < N
        lv = lev_log(min(i+1, N));
        k1 = eom(x_cl,          t,        lv, thrust_fn, mass_fn, drag_fn, g0);
        k2 = eom(x_cl+dt/2*k1,  t+dt/2,  lv, thrust_fn, mass_fn, drag_fn, g0);
        k3 = eom(x_cl+dt/2*k2,  t+dt/2,  lv, thrust_fn, mass_fn, drag_fn, g0);
        k4 = eom(x_cl+dt*k3,    t+dt,    lv, thrust_fn, mass_fn, drag_fn, g0);
        x_cl = x_cl + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        x_cl(1) = max(x_cl(1), 0.0);
    end
end

apogee_cl = max(z_true(1:i_cl_end));

%% ================================================================
%  TRIM TO FLIGHT WINDOW
%% ================================================================
i_end = min(max(i_ol_end, i_cl_end) + 50, N);
idx   = 1:i_end;
tp    = t_vec(idx);

%% ================================================================
%  SUMMARY
%% ================================================================
fprintf('\n  ================================\n');
fprintf('  SIMULATION RESULTS\n');
fprintf('  ================================\n');
fprintf('  Natural apogee (no brakes) : %7.1f m  (%6.0f ft)\n', ...
        apogee_ol, apogee_ol*3.28084);
fprintf('  PID target                 : %7.1f m  (%6.0f ft)\n', ...
        Z_TARGET, Z_TARGET*3.28084);
fprintf('  Achieved apogee (PID+EKF)  : %7.1f m  (%6.0f ft)\n', ...
        apogee_cl, apogee_cl*3.28084);
fprintf('  Error vs target            :   %+5.1f m  (%+.1f ft)\n', ...
        apogee_cl - Z_TARGET, (apogee_cl - Z_TARGET)*3.28084);
fprintf('  Max deployment angle       : %6.1f °\n', max(ang_log));
fprintf('  Max velocity               : %6.1f m/s\n', max(v_true));
fprintf('  Max Mach                   : %6.3f\n', max(Mach_log));
fprintf('  Max drag force             : %6.0f N\n', max(Fd_log));
fprintf('  Max acceleration           : %6.1f m/s²  (%.2f g)\n', ...
        max(accel_log), max(accel_log)/g0);
fprintf('  ================================\n\n');

%% ================================================================
%  FIGURE
%% ================================================================
figure('Name', 'MAGS Airbrake — PID + EKF Closed-Loop Simulation', ...
       'Color', 'w', 'Position', [30 30 1560 930]);

C_ol    = [0.50, 0.50, 0.50];   % open-loop grey
C_cl    = [0.12, 0.47, 0.71];   % closed-loop blue
C_ekf   = [0.84, 0.15, 0.16];   % EKF red
C_band  = [0.80, 0.80, 1.00];   % ±2σ fill
C_green = [0.17, 0.63, 0.17];
C_pur   = [0.58, 0.40, 0.74];
C_teal  = [0.09, 0.75, 0.81];

shade = @(ax_h, tp, mu, s2) fill(ax_h, ...
    [tp, fliplr(tp)], ...
    [mu+2*sqrt(abs(s2)), fliplr(mu-2*sqrt(abs(s2)))], ...
    C_band, 'EdgeColor','none','FaceAlpha',0.45);

% ── 1. Altitude ─────────────────────────────────────────────────────
ax1 = subplot(3,3,1);
shade(ax1, tp, z_est(idx), Pz_var(idx)); hold on;
plot(tp, z_ol(idx),   '--', 'Color', C_ol,  'LineWidth', 1.4);
plot(tp, z_true(idx), '-',  'Color', C_cl,  'LineWidth', 1.8);
plot(tp, z_est(idx),  ':',  'Color', C_ekf, 'LineWidth', 1.4);
yline(apogee_ol, '--', 'Color', C_ol,  'LineWidth', 0.9);
yline(Z_TARGET,  '--', 'Color', C_ekf, 'LineWidth', 1.1, ...
      'Label', sprintf('Target  %.0f m', Z_TARGET), ...
      'LabelHorizontalAlignment','left');
xlabel('Time (s)'); ylabel('Altitude AGL (m)');
title('Altitude'); grid on;
legend('EKF ±2\sigma','No airbrakes','Closed-loop true','EKF est.', ...
       'Location','northwest','FontSize',7);

% ── 2. Velocity ─────────────────────────────────────────────────────
ax2 = subplot(3,3,2);
shade(ax2, tp, v_est(idx), Pv_var(idx)); hold on;
plot(tp, v_ol(idx),   '--', 'Color', C_ol,  'LineWidth', 1.4);
plot(tp, v_true(idx), '-',  'Color', C_cl,  'LineWidth', 1.8);
plot(tp, v_est(idx),  ':',  'Color', C_ekf, 'LineWidth', 1.4);
yline(0, 'k:', 'LineWidth', 0.8);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Vertical Velocity'); grid on;
legend('EKF ±2\sigma','No airbrakes','Closed-loop true','EKF est.', ...
       'Location','best','FontSize',7);

% ── 3. Drag Force ────────────────────────────────────────────────────
ax3 = subplot(3,3,3);
plot(tp, Fd_log(idx), '-', 'Color', C_pur, 'LineWidth', 1.8);
xlabel('Time (s)'); ylabel('Drag Force (N)');
title('Total Drag Force  (closed-loop)'); grid on;

% ── 4. Deployment Angle ──────────────────────────────────────────────
ax4 = subplot(3,3,4);
plot(tp, ang_log(idx), '-', 'Color', C_green, 'LineWidth', 1.8);
ylim([-2, AB_MAX + 8]);
yline(AB_MAX, 'k:', sprintf('Max %.0f°', AB_MAX), 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Deployment Angle (°)');
title('Airbrake Deployment Angle'); grid on;

% ── 5. Mach Number ───────────────────────────────────────────────────
ax5 = subplot(3,3,5);
plot(tp, Mach_log(idx), '-', 'Color', C_teal, 'LineWidth', 1.8);
yline(1.0, 'k--', 'M = 1', 'LabelHorizontalAlignment','left','LineWidth',1);
xlabel('Time (s)'); ylabel('Mach Number');
title('Mach Number'); grid on;

% ── 6. Motor Thrust (m2050x_corrected.eng) ───────────────────────────
ax6 = subplot(3,3,6);
t_fine = linspace(0, t_burn, 500);
F_fine = arrayfun(thrust_fn, t_fine);
plot(t_fine, F_fine, 'k-', 'LineWidth', 1.8); hold on;
plot(T_eng(:,1), T_eng(:,2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor','r');
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Motor Thrust  (m2050x\_corrected.eng)'); grid on;
legend('Interpolated','Data points','Location','northeast','FontSize',7);

% ── 7. Predicted Apogee vs Target ────────────────────────────────────
ax7 = subplot(3,3,7);
plot(tp, zpred_log(idx), '-', 'Color', C_cl, 'LineWidth', 1.8); hold on;
yline(Z_TARGET, 'r--', sprintf('Target  %.0f m', Z_TARGET), ...
      'LabelHorizontalAlignment','left','LineWidth',1.2);
yline(apogee_ol, '--', 'Color', C_ol, 'LineWidth', 0.9, ...
      'Label', sprintf('No-brake  %.0f m', apogee_ol), ...
      'LabelHorizontalAlignment','left');
i_arm = find(lev_log(idx) > 0.01, 1, 'first');
if ~isempty(i_arm)
    xline(tp(i_arm), ':', 'Color',[0.5 0.5 0.5], 'LineWidth',1, ...
          'Label','Brakes armed','LabelOrientation','horizontal');
end
xlabel('Time (s)'); ylabel('Predicted Apogee (m)');
title('Apogee Prediction  (EKF energy-balance)'); grid on;

% ── 8. PID Error & Deployment Level ─────────────────────────────────
ax8 = subplot(3,3,8);
yyaxis left;
plot(tp, pid_err(idx), '-', 'Color', C_cl, 'LineWidth', 1.4);
yline(0, 'k:', 'LineWidth', 0.8);
ylabel('Apogee Error (m)');
ax8.YColor = C_cl;
yyaxis right;
plot(tp, lev_log(idx), '-', 'Color', C_green, 'LineWidth', 1.8);
ylabel('Deployment Level [0–1]');
ylim([-0.05, 1.15]);
ax8.YColor = C_green;
xlabel('Time (s)');
title('PID: Error Signal & Deployment Level'); grid on;
legend('Apogee error','Deployment level','Location','best','FontSize',7);

% ── 9. Net Acceleration ──────────────────────────────────────────────
ax9 = subplot(3,3,9);
yyaxis left;
plot(tp, accel_log(idx), '-', 'Color', C_cl, 'LineWidth', 1.8);
yline(0, 'k:', 'LineWidth', 0.8);
ylabel('Acceleration (m/s²)');
ax9.YColor = C_cl;
yyaxis right;
plot(tp, accel_log(idx)/g0, '--', 'Color', C_ekf, 'LineWidth', 1.4);
ylabel('Acceleration (g)');
ax9.YColor = C_ekf;
xlabel('Time (s)');
title('Net Vertical Acceleration'); grid on;
legend('m/s²','g-force','Location','best','FontSize',7);

linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8,ax9], 'x');
xlim([0, tp(end)]);

sgtitle({ ...
    'MAGS Rocket — Airbrake Closed-Loop Simulation  (PID + EKF)', ...
    sprintf(['m2050x\\_corrected.eng  |  No-brake apogee: %.0f m  ' ...
             '|  PID target: %.0f m  (−100 m)  |  Achieved: %.0f m'], ...
            apogee_ol, Z_TARGET, apogee_cl)}, ...
    'FontSize', 12, 'FontWeight', 'bold');

%% ================================================================
%  LOCAL FUNCTIONS
%% ================================================================
function dxdt = eom(x, t, lev, thrust_fn, mass_fn, drag_fn, g0)
    z  = x(1);  v  = x(2);
    Ft = thrust_fn(t);
    m  = mass_fn(t);
    Fd = drag_fn(max(z, 0), v, lev);
    dxdt = [v; (Ft - Fd*sign(v) - m*g0) / m];
end

function z_ap = predict_apogee(z0, v0, lev, m, drag_fn, g0)
% Numerically integrate the post-burnout coast EOM forward from (z0,v0)
% holding deployment level lev fixed until vertical velocity reaches zero.
% Uses a coarse Euler step — accuracy is sufficient for PID error signal.
    dt_p  = 0.20;   % prediction timestep [s]
    N_max = 400;    % max steps (= 80 s look-ahead, well past any apogee)
    z = z0;  v = v0;
    for k = 1:N_max
        if v <= 0, break; end
        Fd = drag_fn(max(z, 0), v, lev);   % drag magnitude
        a  = (-Fd - m*g0) / m;             % coast decel (v>0, sign(v)=1)
        v  = v + a  * dt_p;
        z  = z + v  * dt_p;
    end
    z_ap = z;
end
