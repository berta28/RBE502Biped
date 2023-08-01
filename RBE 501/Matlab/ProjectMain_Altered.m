clc; clear; close all; cla;
syms(sym('theta',[1 6], 'real'));
syms(sym('L',[1 5], 'real'));

%% Target points defined
%transforms from table coordinates to target postion to space
T00 = [1 0 0 0;
       0 0 1 0;
       0 -1 0 0;
       0 0  0 1;]';
        
p1 = [-175; 125; 0]/1000;
p2 = [175; 125; 0]/1000;
p3 = [0; 150; -200]/1000;
F1 = [0.961 -0.961 0];
F2 = [-0.961 -0.961 0];
F3 = [0 -1.36 0];

F1_norm = F1 / norm(F1);
x_hat = [0 0 1];
y_hat1 = cross(F1_norm,x_hat);
R1= [x_hat;
    y_hat1;
    F1_norm]';

Td1 = [ R1 p1; 0 0 0 1; ];

F2_norm = F2 / norm(F2);
y_hat2 = cross(F2_norm,x_hat);
R2= [x_hat;
    y_hat2;
    F2_norm]';

Td2 = [ R2 p2; 0 0 0 1;];
F3_norm = F3 / norm(F3);
y_hat3 = cross(F3_norm,x_hat);
R3= [x_hat;
    y_hat3;
    F3_norm]';
Td3 = [ R3 p3; 0 0 0 1; ];

valid_rotation_matrix(R1)
valid_rotation_matrix(R2)
valid_rotation_matrix(R3)
Tsd1 = T00 * Td1;
Tsd2 = T00 * Td2;
Tsd3 = T00 * Td3;

valid_rotation_matrix(Tsd1)
valid_rotation_matrix(Tsd2)
valid_rotation_matrix(Tsd3)
%plots the target poses 
figure('Name','Foward with desired poses')
plotFrame(Tsd1,50/1000,3,0.5)
plotFrame(Tsd2,50/1000,3,0.5)
plotFrame(Tsd3,50/1000,3,0.5)

%how far the robot is for the center of the table link lengths
L = [100 250 200 150 0]/1000;
eef = 163/1000;
d2c = 250/1000;
numJoints = 6
%% Question 1 Product of Exponentials
W = sym('W',[6 3]);
q = sym('q',[6 3]);
W(1,:) = [0 0 0];
W(2,:) = [0 1 0];
W(3,:) = [0 1 0];
W(4,:) = [1 0 0];
W(5,:) = [0 1 0];
W(6,:) = [1 0 0];
W(7,:) = [0 -1 0];
q(1,:) = [-d2c 0 0];
q(2,:) = [-d2c 0 L1];
q(3,:) = [-d2c 0 L1+L2];
q(4,:) = [L3-d2c 0 L1+L2];
q(5,:) = [L3+L4-d2c 0 L1+L2];
q(6,:) = [L3+L4+L5-d2c 0 L1+L2];
q(7,:) = [L3+L4+L5-d2c 0 L1+L2-eef];

v = sym('v',[numJoints 3]);
v(1,:) = [0 1 0];
for idx = 2:numJoints 
    v(idx,:) = -cross(W(idx,:),q(idx,:));
end
M = sym('M',[4 4 numJoints]);
M(:,:,1) = [0 1 0 0;
            0 0 1 0;
            1 0 0 0;
            0 0 0 1] + [zeros(3,3) q(1,:)'
                        0 0 0 0];
M(:,:,2) = [0 1 0 0;
            0 0 1 0;
            1 0 0 0;
            0 0 0 1] + [zeros(3,3) q(2,:)'
                        0 0 0 0];
M(:,:,3) = [0 1 0 0;
            0 0 1 0;
            1 0 0 0;
            0 0 0 1] + [zeros(3,3) q(3,:)'
                        0 0 0 0];
M(:,:,4) = [0 0 1 0;
            0 -1 0 0;
            1 0 0 0;
            0 0 0 1] + [zeros(3,3) q(4,:)'
                        0 0 0 0];
M(:,:,5) = [0 1 0 0;
            0 0 1 0;
            1 0 0 0;
            0 0 0 1] + [zeros(3,3) q(5,:)'
                        0 0 0 0];
M(:,:,6) = [0 -1 0 0;
            -1 0 0 0;
            0 0 -1 0;
            0 0 0 1] + [zeros(3,3) q(7,:)'
                        0 0 0 0];

% M(:,:,6) = [0 0 1 0;
%             0 -1 0 0;
%             1 0 0 0;
%             0 0 0 1] + [zeros(3,3) q(6,:)'
%                         0 0 0 0];
M(:,:,7) = [0 -1 0 0;
            -1 0 0 0;
            0 0 -1 0;
            0 0 0 1] + [zeros(3,3) q(7,:)'
                        0 0 0 0];

v_num = subs(v,[L1, L2, L3, L4, L5],L);
M_num = subs(M,[L1, L2, L3, L4, L5],L);

%% Question 2 Forward Kinematics
q = [theta1 theta2 theta3 theta4 theta5 theta6];
q_value = [0 pi/4 -pi/4 0 pi/2 0];
q_home = [0 0 0 0 0 0];
% q_num = subs(q, [theta1 theta2 theta3 theta4 theta5 theta6], q_value);
q_num = subs(q, [theta1 theta2 theta3 theta4 theta5 theta6], q_home);

T = zeros(4,4,numJoints);

% plotLink(T,3);
% plotFrame([1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1],10, 3);
S_num = [W(1:numJoints,:) v_num]';
plotRobot(S_num,M_num,q_num)


%% Inverse Kinematics


q =[p1(3) rand(1,numJoints-1)]';
% q=[100 -pi/2 0 0 0 pi/2]';
% 
q_d1 = ikSolver(Tsd1,S_num,M_num,q);
figure('Name','q1');
plotFrame(Tsd1,50/1000,3,0.5)
plotFrame(Tsd2,50/1000,3,0.5)
plotFrame(Tsd3,50/1000,3,0.5)
plotRobot(S_num,M_num,q_d1)


q =[p2(3) rand(1,numJoints-1)]';
q_d2 = ikSolver(Tsd2,S_num,M_num,q);
figure('Name','q2')
plotFrame(Tsd1,50/1000,3,0.5)
plotFrame(Tsd2,50/1000,3,0.5)
plotFrame(Tsd3,50/1000,3,0.5)
plotRobot(S_num,M_num,q_d2)


figure('Name',"q3")
plotFrame(Tsd1,50/1000,3,0.5)
plotFrame(Tsd2,50/1000,3,0.5)
plotFrame(Tsd3,50/1000,3,0.5)
q =[0 0 0 0 0 0]';
q_d3 = ikSolver(Tsd3,S_num,M_num,q);
plotRobot(S_num,M_num,q_d3)

%% initial force calulation due to gravity of the probe
Tsb = double(fkine(S_num,M_num(:,:,numJoints+1),q_d3));
r = Tsb(1:3,4);
Fsf = [0; 0; .309*-9.81]; % force of gravity from the probe
FsM = cross(r,Fsf);
Fs = [FsM; Fsf];
Js = double(jacob0(S_num,q_d3));
torques = Js'*Fs;

%solve for forces in the body frame
% Rbs = pinv(Tsb(1:3,1:3))
% FbM = [0 0 0]'
% Fbf = Rbs * Fsf;
% Fb = [FbM; Fbf;]
% torqueb = Jb'*Fb

%% Path generation
q = [q(1); wrapToPi(q(2:size(q,1)))];
Thome = fkine(S_num,M_num(:,:,length(q)+1),q);
q_d1 = [q_d1(1); wrapToPi(q_d1(2:size(q_d1,1)))];
q_d2 = [q_d2(1); wrapToPi(q_d2(2:size(q_d2,1)))];
q_d3 = [q_d3(1); wrapToPi(q_d3(2:size(q_d3,1)))];
[ang_q1,vel_q1,acc_q1,t1] =pathPlanJoint(q,q_d1);
[ang_q2,vel_q2,acc_q2,t2] =pathPlanJoint(q_d1,q_d2);
[ang_q3,vel_q3,acc_q3,t3] =pathPlanJoint(q_d2,q_d3);
[ang_q4,vel_q4,acc_q4,t4] =pathPlanJoint(q_d3,q);


%%creaate plots for path plan
plotPathPlan(ang_q1,vel_q1,acc_q1,t1,"Path1")
plotPathPlan(ang_q2,vel_q2,acc_q2,t2,"Path2")
plotPathPlan(ang_q3,vel_q3,acc_q3,t3,"Path3")
plotPathPlan(ang_q4,vel_q4,acc_q4,t4,"Path4")


%Torque calulations
%% 
mass_links = [0.50 0.04 0.11 0.09 0.06 0];
mass_links(6)= mass_links(6) + .309;
motor_wieghts = [3.8 2.7 .42 .6 .42 0];
mass = mass_links + motor_wieghts;
samples = length(t1);
tau1 = zeros(6,samples);
% tool_weight =mass(7) - .5* -9.81;
Fsf= [0; 0; 0;] ;
F1 = T00(1:3,1:3)*(F1 * 9.81)' ;
F2 = T00(1:3,1:3)*(F2 * 9.81)' ;
F3 = T00(1:3,1:3)*(F3 * 9.81)' ;
for x = 1:1:samples
    if x== samples
        tau1(:,x) = motor_torque(M_num,S_num,ang_q1(:,x),vel_q1(:,x),acc_q1(:,x),mass,F1);
    else
        tau1(:,x) = motor_torque(M_num,S_num,ang_q1(:,x),vel_q1(:,x),acc_q1(:,x),mass,Fsf);
    end
    
end
tau2 = zeros(6,samples);
for x = 1:1:samples
    if x== samples
        tau2(:,x) = motor_torque(M_num,S_num,ang_q2(:,x),vel_q2(:,x),acc_q2(:,x),mass,F2);
    else
        tau2(:,x) = motor_torque(M_num,S_num,ang_q2(:,x),vel_q2(:,x),acc_q2(:,x),mass,Fsf);
    end
    
end
tau3 = zeros(6,samples);
for x = 1:1:samples
    if x== samples
        tau3(:,x) = motor_torque(M_num,S_num,ang_q3(:,x),vel_q3(:,x),acc_q3(:,x),mass,F3);
    else
        tau3(:,x) = motor_torque(M_num,S_num,ang_q3(:,x),vel_q3(:,x),acc_q3(:,x),mass,Fsf);
    end
    
end


for x = 1:6
path1_max(x) = max(abs(tau1(x,:))) * 1.1;
path2_max(x) = max(abs(tau2(x,:))) * 1.1;
path3_max(x) = max(abs(tau3(x,:))) * 1.1;
end
path1_max
path2_max
path3_max

figure, plot(t1,tau1(:6,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('Path 1');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'},'Location','southwest','Orientation','horizontal')

figure, plot(t2,tau2(2:6,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('Path 2');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'},'Location','southwest','Orientation','horizontal')

figure, plot(t3,tau3(2:6,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('Path 3');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'},'Location','southwest','Orientation','horizontal')

figure, plot(t1,tau1(1,:));
xlabel('time [s]'), ylabel('force [N]'), title('Path 1');
legend({'Joint 1'},'Location','southwest','Orientation','horizontal')

figure, plot(t2,tau2(1,:));
xlabel('time [s]'), ylabel('force [N]'), title('Path 2');
legend({'Joint 1'},'Location','southwest','Orientation','horizontal')

figure, plot(t3,tau3(1,:));
xlabel('time [s]'), ylabel('force [N]'), title('Path 3');
legend({'Joint 1'},'Location','southwest','Orientation','horizontal')

% close all
% frames1 = animatePath(Tsd1,ang_q1,S_num,M_num);
% frames2 = animatePath(Tsd2,ang_q2,S_num,M_num);
% frames3 = animatePath(Tsd3,ang_q3,S_num,M_num);
% frames4 = animatePath(Thome,ang_q4,S_num,M_num);
% frames = struct([frames1 frames2 frames3 frames4]);
% 
% %% Animation for joint movements
% 
% close all
% figure('Name','Animation')
% axes("Position",[0 0 1 1])
% movie(frames, 2, 5)
%% 

% T_num = fkine(W,v_num,M_num(:,:,6),q_num);
% display(double(T_num));

% %% Question 3 Jacobian
% Wb = [1 0 0;
%       0 -1 0;
%       0 -1 0;
%       0 0 1;
%       0 -1 0;
%       0 0 1];
% qb = [-L1-L2-L3, 0, -L4-L5-L6;
%       -L2-L3, 0, -L4-L5-L6;
%       -L3, 0, -L4-L5-L6;
%       0, 0, -L5-L6;
%       0, 0, -L6;
%       0, 0, 0];
% 
% vb = sym('vb',[6 3]);
% for idx = 1:6
%     vb(idx,:) = -cross(Wb(idx,:),qb(idx,:));
% end
% 
% vb_num = subs(vb,[L0, L1, L2, L3, L4, L5, L6],10e-4*[165 125 270 70 134 168 72]);
% Wb_num = subs(Wb,[L0, L1, L2, L3, L4, L5, L6],10e-4*[165 125 270 70 134 168 72]);
% B = [Wb_num'; vb_num'];
% joint_angle = [pi/4 0 pi/4 0 -pi/4 pi/4];
% Jb = bodyJacobian(B,joint_angle)
% %% Question 4 Analytical Jacobian
% r = Wb_num'*joint_angle';
% Rsb = T_num(1:3,1:3);
% Rsn_norm = norm(Rsb);
% Rsb_skew = [0 -Rsb(3) Rsb(2);
%             Rsb(3) 0 -Rsb(1);
%             -Rsb(2) Rsb(1) 0];
% A = eye(3) - ((1 -cos(Rsn_norm))/Rsn_norm^2)*Rsb_skew + ((Rsn_norm - sin(Rsn_norm))/Rsn_norm^3)*Rsb_skew^2;
% Ja = double([inv(A) zeros(3,3);
%       zeros(3,3) Rsb] * Jb);
% %% Qustion 5 Manipulability and Singularities
% % Singularity test
% JbRank = rank(Jb);
% if JbRank < 6
%     fprintf("Jacobian is not full rank. Robot is at singularity configuration. \n");
% else
%     fprintf("Jacobian is Full rank. Robot is not at singularity configuration. \n");
% end
% % Manipulability
% Jbw = Jb(1:3,:);
% Jbv = Jb(4:6,:);
% 
% Aw = Jbw*Jbw';
% Av = Jbv*Jbv';
% 
% AwEigValues = eig(Aw);
% AvEigValues = eig(Av);
% muAw = max(AwEigValues)/min(AwEigValues);
% muAv = max(AvEigValues)/min(AvEigValues);
% %% Question 6 Statics
% Fb = [0 0 0 -100 0 0]';
% tau = Jb'*Fb;
% 
% %% Question 7 Inverse Kinematics - Twist in the Body Frame
% Tsd = [sqrt(2)/2 -sqrt(2)/2 0 0.2;
%        sqrt(2)/2 sqrt(2)/2 0 0.2;
%        0 0 1 0.5;
%        0 0 0 1];
% Tbd = inv(T_num)*Tsd;
% theta_twist = acos(0.5*(trace(Tbd(1:3,1:3)) - 1));
% omega_skew = 1/(2*sin(theta_twist))*(Tbd(1:3,1:3)-Tbd(1:3,1:3)');
% G_inv = 1/theta_twist * eye(3,3) - 0.5*omega_skew + (1/theta_twist - 0.5*cos(theta_twist/2))*omega_skew^2;
% v_twist = G_inv*Tbd(1:3,4);
% omega_twist = [-omega_skew(2,3), omega_skew(1,3), -omega_skew(1,2)]';
% twist = [omega_twist;v_twist]*theta_twist;
% 
% %% Question 8 Inverse Kinematics - Newton Raphson Step
% delta_theta = inv(Jb) * twist;
% theta_new = joint_angle' - delta_theta;
