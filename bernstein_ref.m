function r = bernstein_ref(t, Tf, r_points)
% r = bernstein_ref(t, Tf, r_points)
% Bernstein polynomial reference on [0, Tf]
%
% Inputs:
%   t        : time (sec)
%   Tf       : final time (sec)
%   r_points : control points [r0 r1 ... rN] (length N+1)
%
% Output:
%   r : reference value at time t

N = numel(r_points) - 1;

% clamp time to [0, Tf]
if t < 0
    tau = 0;
elseif t > Tf
    tau = 1;
else
    tau = t / Tf;
end

r = 0;
for i = 0:N
    bi = nchoosek(N,i) * (tau^i) * ((1 - tau)^(N - i));
    r = r + r_points(i+1) * bi;
end
end
