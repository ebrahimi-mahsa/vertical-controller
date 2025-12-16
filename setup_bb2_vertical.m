%vertical simulink model based on the data provided by the Prof from under
%water vehicle with (lqr)- bernestein and lqr

clear; clc;

%% 1) Plant matrices obtained from  the refrenced article
A = [ -9.6e-13  5.0e+00   1.0e+00   1.1e-10
       1.4e-12 -5.1e-14 -1.2e-10   1.0e+00
       8.4e-04 -3.04e-07 -1.0e-01 -2.7e+00
       4.4e-06 -1.5e-02 -5.0e-03 -2.4e-01 ];

B = [ -1.69e-12 0 0
      -7.22e-13 0 0
      -2.61e-03 0 0
       3.30e-04 0 0 ];

Bv = B(:,1);             % delta_v
C  = [1 0 0 0];          % output: depth z
D  = 0;

%% 2) Bernstein order (start with order3)
N = 3;                    
neta = N + 1;

S = [0 1 0 0;
     0 0 1 0;
     0 0 0 1;
     0 0 0 0];

Upsilon = [0;0;0;1];

%% 3) Augmented system for LQR
Az = [A               zeros(4,neta);
      Upsilon*C       S];

Bz = [Bv;
      zeros(neta,1)];

%% 4) LQR weights (safe starter values)
%Qx = diag([100, 10, 1, 1]);      % this is not good result
                                  
%Qe = 10*eye(neta);              
%Qz = blkdiag(Qx, Qe);
%R  = 1;                          

Qx = diag([10, 1, 0.1, 0.1]);     % [z theta w q]
Qe = 1e-4*eye(neta);              % eta weights
Qz = blkdiag(Qx, Qe);

R  = 1e4;                         

%% 5) LQR gain and split into Kx, Keta
Kz   = lqr(Az, Bz, Qz, R);
Kx   = Kz(:, 1:4);
Keta = Kz(:, 5:end);

%% 6) check stability with eighenvalues of closed loop
eig_cl = eig(Az - Bz*Kz);
disp('Closed-loop eigenvalues (Az - Bz*Kz):');
disp(eig_cl);

%% 7) Reference parameters for bernestein block
Tf = 200;                        % seconds 
r_points = [-100 -100 -50 -30];  % acccording to prof refrences



Cout = [C; eye(4)];      % output = [y; x]
Dout = [0; zeros(4,1)];
