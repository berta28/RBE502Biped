clc
clear

%Link lengths
L1 = 900;
L2 = 75;
L3 = 250;
L4 = 150;
L5 = 150;
L6 = 50;
sensor_length = 163;

%mass per link
mass1 = 0.50;
mass2 = 0.03;
mass3 = 0.11;
mass4 = 0.06;
mass5 = 0.06;
mass6 = 0.02;
sensor_mass = 0.19;

% M matrices
M_1 = [0 1 0 0; 0 0 1 0; 1 0 0 0; 0 0 0 1];
M_2 = [0 1 0 0; 0 0 1 0; 1 0 0 L1; 0 0 0 1];
M_3 = [0 1 0 0; 0 0 1 0; 1 0 0 L1+L2; 0 0 0 1];
M_4 = [0 0 1 L3; 0 -1 0 0; 1 0 0 L1+L2; 0 0 0 1];
M_5 = [0 1 0 L3+L4; 0 0 1 0; 1 0 0 L1+L2; 0 0 0 1];
M_6 = [0 0 1 L3+L4+L5; 0 -1 0 0; 1 0 0 L1+L2; 0 0 0 1];

% screw axes
W_1 = [0; 0; 0];
V_1 = [0; 1; 0];
W_2 = [0; 1; 0];
V_2 = [-L1; 0; 0];
W_3 = [0; 1; 0];
V_3 = [-L1-L2; 0; 0];
W_4 = [1; 0; 0];
V_4 = [0; L1+L2; 0];
W_5 = [0; 1; 0];
V_5 = [-L1-L2; 0; L3+L4];
W_6 = [1; 0; 0];
V_6 = [0; L1+L2; 0];

%position1
theta = [0 -0.959 1.1543 -21.9911 -0.59 23.5619];
thetadot = [1 1 1 1 1 1];
thetaddot = [1 1 1 1 1 1];

%Joint transformations
eST1 = poejoint(W_1, V_1, theta(1));
eST2 = poejoint(W_2, V_2, theta(2));
eST3 = poejoint(W_3, V_3, theta(3));
eST4 = poejoint(W_4, V_4, theta(4));
eST5 = poejoint(W_5, V_5, theta(5));
eST6 = poejoint(W_6, V_6, theta(6));

%Forward kinematics
Tsb = eST1*eST2*eST3*eST4*eST5*eST6*M_6

%Inverse kinematics
S_1 = [W_1; V_1];
S_2 = [W_2; V_2];
S_3 = [W_3; V_3];
S_4 = [W_4; V_4];
S_5 = [W_5; V_5];
S_6 = [W_6; V_6];

Js = [S_1, adjoint(S_1, theta(1))*S_2, adjoint(S_1, theta(1))*adjoint(S_2, theta(2))*S_3, ...
        adjoint(S_1, theta(1))*adjoint(S_2, theta(2))*adjoint(S_3, theta(3))*S_4, ...
        adjoint(S_1, theta(1))*adjoint(S_2, theta(2))*adjoint(S_3, theta(3))*adjoint(S_4, theta(4))*S_5, ...
        adjoint(S_1, theta(1))*adjoint(S_2, theta(2))*adjoint(S_3, theta(3))*adjoint(S_4, theta(4))*adjoint(S_5, theta(5))*S_6];

Tbs = Tsb^(-1)

Jb = adjT(Tbs)*Js

%Static Forces
Rbs = Tbs(1:3, 1:3)

%position1
Fb1 = [0 0 0 0.961 -0.961 0]';
Tau1 = Jb'*Fb1

%Dynamics: Recursive Newton Euler Algorithm (Forward Iteration)
M01 = M_1
M12 = M_1^(-1)*M_2
M23 = M_2^(-1)*M_3
M34 = M_3^(-1)*M_4
M45 = M_4^(-1)*M_5
M56 = M_5^(-1)*M_6

S = [S_1 S_2 S_3 S_4 S_5 S_6]

A1 = adjT(M_1^(-1))*S_1;
A2 = adjT(M_2^(-1))*S_2;
A3 = adjT(M_3^(-1))*S_3;
A4 = adjT(M_4^(-1))*S_4;
A5 = adjT(M_5^(-1))*S_5;
A6 = adjT(M_6^(-1))*S_6;
A = [A1 A2 A3 A4 A5 A6]

T10 = M01^(-1);
T21 = M12^(-1);
T32 = M23^(-1);
T43 = M34^(-1);
T54 = M45^(-1);
T65 = M56^(-1);

Ad1 = ad(T10);
Ad2 = ad(T21);
Ad3 = ad(T32);
Ad4 = ad(T43);
Ad5 = ad(T54);
Ad6 = ad(T65);

V1 = A1*thetadot(1);
V2 = A2*thetadot(2) + adjT(T21)*V1;
V3 = A3*thetadot(3) + adjT(T32)*V2;
V4 = A4*thetadot(4) + adjT(T43)*V3;
V5 = A5*thetadot(5) + adjT(T54)*V4;
V6 = A6*thetadot(6) + adjT(T65)*V5;
V = [V1 V2 V3 V4 V5 V6]     %joint velocities

Vd0 = [0 0 0 0 9.8 0]';
Vd1 = A1*thetaddot(1) + ad(V1)*A1*thetadot(1) + adjT(T10)*Vd0;
Vd2 = A2*thetaddot(2) + ad(V2)*A2*thetadot(2) + adjT(T21)*Vd1;
Vd3 = A3*thetaddot(3) + ad(V2)*A2*thetadot(3) + adjT(T21)*Vd2;
Vd4 = A4*thetaddot(4) + ad(V2)*A2*thetadot(4) + adjT(T21)*Vd3;
Vd5 = A5*thetaddot(5) + ad(V2)*A2*thetadot(5) + adjT(T21)*Vd4;
Vd6 = A6*thetaddot(6) + ad(V2)*A2*thetadot(6) + adjT(T21)*Vd5;
Vd = [Vd1 Vd2 Vd3 Vd4 Vd5 Vd6]      %joint accelerations

%Dynamics: Recursive Newton Euler Algorithm (Backward Iteration)
G1 = [zeros(3,3), zeros(3,3); zeros(3,3), mass1*eye(3)]
G2 = [zeros(3,3), zeros(3,3); zeros(3,3), mass2*eye(3)]
G3 = [zeros(3,3), zeros(3,3); zeros(3,3), mass3*eye(3)]
G4 = [zeros(3,3), zeros(3,3); zeros(3,3), mass4*eye(3)]
G5 = [zeros(3,3), zeros(3,3); zeros(3,3), mass5*eye(3)]
G6 = [zeros(3,3), zeros(3,3); zeros(3,3), mass6*eye(3)]

%position1
F7 = Fb1;
F6 = adjT(T32)'*F7 + G2*Vd6 - ad(V6)'*G6*V5;
F5 = adjT(T32)'*F6 + G2*Vd5 - ad(V5)'*G5*V4;
F4 = adjT(T32)'*F5 + G2*Vd4 - ad(V4)'*G4*V3;
F3 = adjT(T32)'*F4 + G2*Vd3 - ad(V3)'*G3*V3;
F2 = adjT(T32)'*F3 + G2*Vd2 - ad(V2)'*G2*V1;
F1 = adjT(T21)'*F2 + G1*Vd1;
F = [F1 F2 F3 F4 F5 F6]

tau6 = F6'*A6;
tau5 = F5'*A5;
tau4 = F4'*A4;
tau3 = F3'*A3;
tau2 = F2'*A2;
tau1 = F1'*A1;
tau = [tau1 tau2 tau3 tau4 tau5 tau6]   %joint torques


function trans = adjoint(S, theta)
    skewOmega = [0, -S(3), S(2); S(3), 0, -S(1); -S(2), S(1), 0];
    R = eye(3,3) + sin(theta)*skewOmega + (1-cos(theta))*skewOmega^2;
    v = [S(4) S(5) S(6)]';
    p = (eye(3,3)*theta + (1-cos(theta))*skewOmega + ((theta-sin(theta))*skewOmega^2))*v;
    skewP = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    trans = [R, zeros(3,3); skewP*R, R];
end
function [screwAxes] = poejoint(w, v, theta)
    skewOmega = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
    R = eye(3,3) + sin(theta)*skewOmega + (1-cos(theta))*skewOmega^2; %Rodrigues' formula
    d = (eye(3,3)*theta + (1-cos(theta))*skewOmega + (theta-sin(theta))*skewOmega^2)*v;

    screwAxes = [R, d; 0, 0, 0, 1];
end

function adT = adjT(T)
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    skewP = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    adT = [R, zeros(3,3); skewP*R, R];
end

function adV = ad(V)
    v1 = [V(4) V(5) V(6)];
    w1 = [V(1) V(2) V(3)];
    skewV = [0, -v1(3), v1(2); v1(3), 0, -v1(1); -v1(2), v1(1), 0];
    skewW = [0, -w1(3), w1(2); w1(3), 0, -w1(1); -w1(2), w1(1), 0];
    adV = [skewW, zeros(3,3); skewV, skewW];
end