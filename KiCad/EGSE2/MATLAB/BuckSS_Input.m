% --- INPUT CONTROL (COMP2) STATE-SPACE FOR BUCK MODE ---
% Simulating Control-to-Input (VINDPM) for 15V to 8.4V Buck

% System Operating Point
L = 2.2e-6; 
C_out = 110.1e-6;   % Output capacitance from previous script
C_in = 22e-6;       % Estimated bulk input capacitance on your board
Vg = 20;            % Ideal open-circuit source voltage (must be > Vout)
Vout = 8.4;         % Target output voltage
R_load = 40/Vout;   % Equivalent DC load (8.4V at 40W charge)

Duty = Vout / Vg;   % Buck duty cycle

% Steady state approximations
I_L = Vout / R_load;  % In buck, average inductor current equals output current (2A)
V_Cin = Vg;           % Input capacitor voltage

% --- TOGGLE THIS VARIABLE to see the poles shift ---
R_in = 0.5;  % Source impedance (cables + power supply internal resistance)

% 3x3 State-Space Matrices (x = [i_L; v_Cin; v_Cout])
A = [ 0            ,  Duty/L           , -1/L ;
     -Duty/C_in    , -1/(R_in * C_in)  ,  0   ;
      1/C_out      ,  0                , -1/(R_load * C_out) ];

% Notice the specific buck mode changes here, specifically -I_L/C_in
B = [ V_Cin/L ; 
     -I_L/C_in ; 
      0 ];

% C matrix isolates the 2nd state variable: input capacitor voltage (v_Cin)
C_mat = [ 0, 1, 0 ];
D_mat = 0;

sys_input_buck = ss(A, B, C_mat, D_mat);
sys_input_buck.InputName = {'Duty Cycle'};
sys_input_buck.OutputName = {'v_Cin'};

% --- TI's Recommended COMP2 Compensator (Fast) ---
gm = 250e-6;
R1_ti = 10000;
C1_ti = 680e-12;
C2_ti = 15e-12;

num_ti = [gm * R1_ti * C1_ti, gm];
den_ti = [R1_ti * C1_ti * C2_ti, C1_ti + C2_ti, 0];
C_comp2_ti = tf(num_ti, den_ti);

L_loop_ti = C_comp2_ti * sys_input_buck;

% --- The "Slow" Robust COMP2 Compensator ---
R1_slow = 40200;
C1_slow = 10e-9; % 10nF PI controller

num_slow = [gm * R1_slow * C1_slow, gm];
den_slow = [0, C1_slow, 0]; % s^2 term is 0 because C2 is omitted
C_comp2_slow = tf(num_slow, den_slow);

L_loop_slow = C_comp2_slow * sys_input_buck;


% --- Plotting ---
% figure(1); margin(L_loop_ti); title('Buck Input Control - TI Fast');
% figure(2); margin(L_loop_slow); title('Buck Input Control - Slow PI');