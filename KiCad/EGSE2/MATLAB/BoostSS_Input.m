% --- INPUT CONTROL (COMP2) STATE-SPACE ---
% Simulating Control-to-Input (VINDPM) for 5V to 8.4V Boost

% System Operating Point
L = 2.2e-6; 
C_out = 110.1e-6;   % Output capacitance from previous script
C_in = 22e-6;       % Estimated bulk input capacitance on your board
Vg = 5;             % Ideal open-circuit source voltage
Vout = 8.4;         % Target output voltage
R_load = 40/Vout;   % Equivalent DC load (8.4V at 40W charge)

Duty = 1 - (Vg / Vout);
D_prime = 1 - Duty;

% Steady state approximations for B matrix
I_L = (Vout^2 / R_load) / Vg;  % Approx 3.36A inductor current
V_Co = Vout;

% --- TOGGLE THIS VARIABLE to see the poles shift ---
% 0.5 ohms = long/thin wire or weak supply
% 0.05 ohms = short/thick wire or stiff bench supply
R_in = 0.5;  

% 3x3 State-Space Matrices (x = [i_L; v_Cin; v_Cout])
A = [ 0            ,  1/L              , -D_prime/L ;
     -1/C_in       , -1/(R_in * C_in)  ,  0         ;
      D_prime/C_out,  0                , -1/(R_load * C_out) ];

B = [ V_Co/L ; 
      0      ; 
     -I_L/C_out ];

% C matrix isolates the 2nd state variable: input capacitor voltage (v_Cin)
C_mat = [ 0, 1, 0 ];
D_mat = 0;

sys_input = ss(A, B, C_mat, D_mat);
sys_input.InputName = {'Duty Cycle'};
sys_input.OutputName = {'v_Cin'};

% --- TI's Recommended COMP2 Compensator ---
gm = 250e-6;
R1_ti = 10000;
C1_ti = 680e-12;
C2_ti = 15e-12;

num_ti = [gm * R1_ti * C1_ti, gm];
den_ti = [R1_ti * C1_ti * C2_ti, C1_ti + C2_ti, 0];
C_comp2_ti = tf(num_ti, den_ti);