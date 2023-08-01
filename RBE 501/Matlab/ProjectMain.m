clc; clear; close all; cla;
syms(sym('theta',[1 6], 'real'));
syms(sym('L',[1 5], 'real'));
sysm q [1,3]



% RBE 501 – Robot Dynamics Fall 2022
% Final Exam
% Assigned: 12/5/2022
% Due: 12/11/2022 by 11:59PM
% This exam applies the principles learned during the first and second half of the semester to the dynamic 
% modeling of two manipulators. Questions 1-4 apply to the SCARA arm of figure 1.  Question 5 and 6 apply 
% to the 2DOF RP manipulator shown in figure 2. Upload a PDF of your answers and show your work. You 
% are allowed (and likely need) to use Matlab to solve the problems, you will also upload your Matlab code. 
% Questions 1-5 are worth 4 points each and question 6 is worth 5 points. 
%  
% Figure 1: A Typical Scara arm (left) and a diagram of it of its links (right)
% L1 = 0.3m m1 = 1.5kg
% L2 = 0.3m m2 = 1.5kg
% L3 = 0.1m m3 = 0.5kg
% L4 = 0.1m m4 = 0.5kg
% L0 = 0.5m
% The center of mass of links 1, 2, and 3 are located halfway along their length. The center of mass of link 
% 4 is located at the center of the gripper (i.e. at the body frame). Model links 1 and 2 as rectangular 
% parallelepipeds with the dimensions shown above. Model links 3 and 4 as point masses. Consider the 
% center of Links 1 and 2 to be at the same z coordinate equal to L0+L3+L4. 

% (4pts) Question 1 Newton Euler Dynamics - Robot Description: 
% Consider the configuration shown in figure 1 to be the home configuration of the robot, write the 4X4 
% home configuration matrix of the body frame, 𝑀, and for the center of mass of each link, 
% 𝑀1, 𝑀2, 𝑀3, 𝑎𝑛𝑑 𝑀4. Fill out the table listed below for the screw axes in the space frame. Write the 
% spatial inertia matrix of each link 𝐺1, 𝐺2, 𝐺3, 𝑎𝑛𝑑 𝐺4. 
% 𝒊 𝝎𝒊 𝒗𝒊
% 1
% 2
% 3
% 4
% 𝑀 = [ ]
% 𝑀1 = [ ]               𝑀2 = [ ]
% 𝑀3 = [ ]                𝑀4 = [ ]
% 𝐺1 = [ ]               𝐺2 = [ ]
% 𝐺3 = [ ]                𝐺4 = [ ]

W = sym('W',[num_joint 3]);
q = sym('q',[num_joint 3]);
W(1,:) = [0 0 1];
W(2,:) = [0 0 0];
W(3,:) = [0 0 0];
W(4,:) = [0 0 0];

q(1,:) = [0 0 0];
q(2,:) = [-d2c 0 L1];
q(3,:) = [-d2c 0 L1+L2];
q(4,:) = [L3-d2c 0 L1+L2];

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





valid_rotation_matrix(R1)
valid_rotation_matrix(R2)
valid_rotation_matrix(R3)

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
Tsb = double(fkine(S_num,M_num(:,:,numJoints),q_d3));
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
Thome = fkine(S_num,M_num(:,:,numJoints),q);
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
% mass = [0.50 0.03 0.11 0.06 0.06 0.02 0.31];
mass = [1 0.53 0.61 0.56 0.56 0.52 0.81];
samples = length(t1);
tau1 = zeros(6,samples);
tool_weight =[0; mass(7) - .5*-9.81; 0];
Fsf= T00(1:3,1:3)* tool_weight;
F1 = T00(1:3,1:3)*(F1'* 9.81+tool_weight);
F2 = T00(1:3,1:3)*(F2'* 9.81+tool_weight) ;
F3 = T00(1:3,1:3)*(F3'* 9.81+tool_weight) ;
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
path1_max(x) = max(tau1(x,:));
path2_max(x) = max(tau2(x,:));
path3_max(x) = max(tau3(x,:));
end
path1_max
path2_max
path3_max

figure, plot(t1,tau1(1:6,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('path1');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'},'Location','northwest','Orientation','horizontal')

figure, plot(t2,tau2(1:6,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('path2');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'},'Location','northwest','Orientation','horizontal')

figure, plot(t3,tau3(1:6,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('path3');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'})

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
