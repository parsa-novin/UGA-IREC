% Operating Setup Variables
Rl = 0.0251 + 0.025;        %Inductor DCR + Fet RdsON
L = 2.2e-6;   %Main Inductor
Rload = 0.05;     %Load Resistance 
Duty = 0.4312;      %Buck Duty cycle
Vin = 20;

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

R_sum = Rload + Rc_eqv;

% Matrix Setup
A = [-((Rl + (Rload * Rc_eqv)/R_sum)/L)  ,   -(Rload / (L * R_sum));
       Rload/(C_eqv + R_sum)             ,   -(1/(R_sum * C_eqv))];

B = [Vin/L ,     0;
      0  , -1/C_eqv];

C = [(Rload * Rc_eqv)/R_sum , Rload/R_sum ;
              1                    0     ];

D = [0, 0;
     0  0];

% TI compensator
gm = 250e-6;
num_ti = [(gm*7.236e-05), gm];
den_ti = [ 2.38788e-15, 1.833e-09, 0];
C_ti = tf(num_ti, den_ti);

% Calculate the system's state-space representation
sys_buck = ss(A, B, C, D);
sys_buck.InputName = {'Duty Cycle', 'Load Disturbance'};
sys_buck.OutputName = {'Vout'};

G_cv = sys_buck(1,1);
G_cc = sys_buck(2,1);
G_cv.OutputName = 'VoltageLoop';
G_cc.OutputName = 'CurrentLoop';
