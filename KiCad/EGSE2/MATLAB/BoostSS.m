% --- BOOST CONFIGURATION FOR BQ25713 (Vin = 5V, Vout = 8.4V) ---
% Separating DC Operating Point from AC Dynamic Load

% Operating Setup Variables
Rl = 0.0251 + 0.025;   % Inductor DCR + Fet RdsON
L = 2.2e-6;            % Main Inductor
Vin = 5;               % Worst-case minimum input
Vout_target = 8.4;

% Define your target charge current here (e.g., 2 Amps)
I_charge = (40/Vout_target);          

% The DC resistance determines the steady-state operating point (IL, VC)
R_dc = Vout_target / I_charge;  

% The AC resistance models the battery's internal impedance at high frequency
R_ac = 0.05;

Duty = 1 - (Vin / Vout_target); 
D_prime = 1 - Duty;

% Capacitor Setup 
C_elec = 100e-6;
C_cer1 = 10e-6;
C_cer2 = 0.1e-6;

% ESR of Caps (At Fsw = 800kHz)
ESR_elec = 0.26;
ESR_cer1 = 0.002;
ESR_cer2 = 0.030;

% Equivalencies
C_eqv = C_elec + C_cer1 + C_cer2;
Rc_eqv = 1/((1/ESR_elec) + (1/ESR_cer1) + (1/ESR_cer2));

% --- 1. Calculate DC Steady-State Operating Point ---
% Use R_dc to find the steady-state inductor current and capacitor voltage
R_sum_dc = R_dc + Rc_eqv;
Rp_dc = (R_dc * Rc_eqv) / R_sum_dc;

A_dc = [-(Rl + D_prime * Rp_dc)/L ,          -(D_prime * R_dc)/(L * R_sum_dc);
         (D_prime * R_dc)/(C_eqv * R_sum_dc), -(1 / (R_sum_dc * C_eqv))];

X_ss = -inv(A_dc) * [Vin/L; 0];
I_L = X_ss(1);
V_C = X_ss(2);

% --- 2. Calculate Small-Signal AC Matrices ---
% Use R_ac because the battery's bulk capacitance shorts out at high AC frequencies
R_sum_ac = R_ac + Rc_eqv;
Rp_ac = (R_ac * Rc_eqv) / R_sum_ac;

% State Matrix (A) now relies on the AC impedance
A = [-(Rl + D_prime * Rp_ac)/L ,          -(D_prime * R_ac)/(L * R_sum_ac);
      (D_prime * R_ac)/(C_eqv * R_sum_ac), -(1 / (R_sum_ac * C_eqv))];

% Input Matrix (B) combines AC impedance with DC operating points (I_L, V_C)
B = [ (Rp_ac * I_L + (R_ac * V_C)/R_sum_ac)/L ,  0;
     -(R_ac * I_L)/(C_eqv * R_sum_ac)         , -1/C_eqv];

C = [ D_prime * Rp_ac, R_ac/R_sum_ac ;
      1              , 0             ];

D = [ -Rp_ac * I_L   , 0;
      0              , 0];

% Calculate the system's state-space representation
sys_boost = ss(A, B, C, D);
sys_boost.InputName = {'Duty Cycle', 'Load Disturbance'};
sys_boost.OutputName = {'Vout', 'IL'};

% --- 3. Evaluate your TI Compensator ---
gm = 250e-6;
num_ti = [(gm*7.236e-05), gm];
den_ti = [ 2.38788e-15, 1.833e-09, 0];
C_ti = tf(num_ti, den_ti);

B_cv = sys_boost(1,1);
B_cc = sys_boost(2,1);

%Compensator Setup
R1 = 237;
C1 = 1.8e-7;
C2 = 5.6e-9;
gm = 250e-6;

C_new_num = [gm * R1 * C1, gm];
C_new_den = [R1 * C1 * C2, C1+C2, 0];

C_new = tf(C_new_num, C_new_den);

% Uncomment below to generate your Bode plot and check phase/gain margins
% margin(L_loop);