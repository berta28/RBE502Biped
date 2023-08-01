function coeff = quinticpoly(t0,tf,q0,qf,qd0,qdf,qdd0,qddf)

% This function generates quintic polynomial trajectories coefficients.
%Inputs
%The following scalar inputs:
% t0, tf the instants when motion starts and stops, 
% q0 and qf , representing the initial and final configurations;
% qd0 and qddf, representing the initial and final velocities;
% qdd0  and qddf, representing the initial and final accelerations.

%returns a 1x6 vector with the six coefficients of the polynomial trajectory.

coeff = zeros(1,6);


A = [
    t0^5      t0^4      t0^3      t0^2     t0     1;
    5*t0^4    4*t0^3    3*t0^2    2*t0     1      0;
    20*t0^3   12*t0^2   6*t0    2        0      0;
    tf^5      tf^4      tf^3      tf^2     tf     1;
    5*tf^4    4*tf^3    3*tf^2    2*tf     1      0;
    20*tf^3   12*tf^2   6*tf    2        0      0;
    ];


B = [q0,qd0,qdd0,qf,qdf,qddf]';
coeff = linsolve(A,B);
coeff = flip(coeff);

end