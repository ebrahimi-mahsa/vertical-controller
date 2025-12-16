% test Lqr weights
% z = [x; eta], so output depth is y = [C 0]*z
n = size(A,1);
m = neta;
Cz = [C, zeros(size(C,1), m)];   % depth output from augmented state


r = -50;  % desired depth
t = 0:0.1:200;

% baseline weights
Qx_base = diag([10, 1, 0.1, 0.1]);
Qe_base = 1e-4*eye(m);
R_base  = 1e4;


tests = {
    'Base',          Qx_base,                 Qe_base,                 R_base;
    'More depth',    diag([50,1,0.1,0.1]),     Qe_base,                 R_base;
    'More eta',      Qx_base,                 1e-3*eye(m),             R_base;
    'Less R',        Qx_base,                 Qe_base,                 1e3;
    'More R',        Qx_base,                 Qe_base,                 1e5;
};

figure; hold on; grid on; title('Depth response y(t)');
legendEntries = {};

for i=1:size(tests,1)
    name = tests{i,1};
    Qx   = tests{i,2};
    Qe   = tests{i,3};
    R    = tests{i,4};

    Qz = blkdiag(Qx,Qe);
    Kz = lqr(Az,Bz,Qz,R);


    Br = [zeros(n,1); -Upsilon
        ];   % upsilon is m×1

    sysCL = ss(Az - Bz*Kz, Br, Cz, 0);
    y = lsim(sysCL, r*ones(size(t)), t);

    plot(t, y, 'DisplayName', name);
    legendEntries{end+1} = name;
end

legend(legendEntries, 'Location','best');
