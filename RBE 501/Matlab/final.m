clc; clear; close all; cla;
% syms(sym('theta',[1 6], 'real'));
% syms(sym('L',[1 5], 'real'));
% syms L0;

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
L1 = 0.3; %m 
m1 = 1.5; %kg
L2 = 0.3; %m 
m2 = 1.5; %kg
L3 = 0.1; %m 
m3 = 0.5; %kg
L4 = 0.1; %m 
m4 = 0.5; %kg
L0 = 0.5; %m
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
L = [0.3 0.3 0.1 0.1];
m = [1.5 1.5 0.5 0.5];
% v_num = subs(v,[L1, L2, L3, L4, L5],L);
% M_num = subs(M,[L1, L2, L3, L4, L5],L);
numJoints = 4 ;
W = sym('W',[numJoints 3]);
q = sym('q',[numJoints 3]);
W(1,:) = [0 0 1];
W(2,:) = [0 0 1];
W(3,:) = [0 0 1];
W(4,:) = [0 0 0];

q(1,:) = [0 0 L0+L3+L4];
q(2,:) = [0 L1 L0+L3+L4];
q(3,:) = [0 L1+L2 L0+L4];
q(4,:) = [0 L1+L2 L0];


% v = sym('v',[numJoints 3]);
v(4,:) = [0 0 1];
for idx = 2:numJoints 
    v(idx,:) = -cross(W(idx,:),q(idx,:));
end
v(4,:) = [0 0 1];
S= [W v]';
%create M for the center of each link
% M = sym('M',[4 4 numJoints]);
M(:,:,1) = [1 0 0 0;
            0 1 0 L1/2;
            0 0 1 L0+L3+L4;
            0 0 0 1]; 
M(:,:,2) = [1 0 0 0;
            0 1 0 L1+L2/2;
            0 0 1 L0+L3+L4;
            0 0 0 1];

M(:,:,3) = [1 0 0 0;
            0 1 0 L1+L2;
            0 0 1 L0+L3/2+L4;
            0 0 0 1];
% does variable joint 4 affect the matrix?
M(:,:,4) = [1 0 0 0;
            0 1 0 L1+L2;
            0 0 1 L0;
            0 0 0 1];


%Define spacial matrix G

G = zeros(6,6,numJoints);
tau = zeros(numJoints,1);
% Ii rectangularparallelepiped
l=.1;
h = .05;
w = .3;
Ii=zeros(3,3,numJoints);
for i = 1:numJoints
    
    if i > 3
        Ii(:,:,i) = zeros(3,3);
    else
        Ixx = m(i)*(w^2 + h^2)/12;
        Iyy = m(i)*(l^2 + h^2)/12;
        Izz = m(i)*(l^2 + w^2)/12;
        Ii(:,:,i) = [
            Ixx 0 0;
            0    Iyy  0;
            0     0    Izz;];
    end

end
for i = 1:numJoints
    G(:,:,i) = [Ii(:,:,i)          zeros(3,3);
      zeros(3,3)  m(i)*eye(3,3);];
end



%% 

% (4pts) Question 2 Newton Euler Dynamics - Pre-calculations: 
% Calculate the screw axes of each joint in their local link frames, A1, A2, A3, and A4. Define the 
% transformation matrices between adjacent links in the home configuration, M01, M12, M23, and M34 :
% 𝐴1 =    []    ;       𝐴2 =  []     ;       𝐴3 =  []    ;     𝐴4 =  []
% 𝑀01 = [ ]               𝑀12 = [ ]
% 𝑀23 = [ ]                𝑀34 = [ ]

A = zeros(6,numJoints);
for i = 1:numJoints
    A(:,i) = adjoint(M(:,:,i)^-1)*S(:,i);
end
M0 = eye(4,4);
for i= numJoints
    if i == 1
       
        Mi(:,:,i) = M(:,:,i)^-1 * M0;
    else
        Mi(:,:,i) = M(:,:,i)^-1 * M(:,:,i-1);
    end
end
MAT = sym('MAT',[3 4])
%% 


% (4pts) Question 3 Newton Euler Dynamics – Joint Trajectories:
% Consider the robot smoothly moving in 3 steps starting from its home position to pick up an object. 
%  Step 1: Joints 1 and 2 rotate from 𝜃1 = 𝜃2 = 0 𝑟𝑎𝑑 𝑡𝑜 𝜃1 = 𝜃2 = 𝜋
% 2𝑟𝑎𝑑 . 
theta0 = [0 0 0 0];

theta1 = [pi/2 pi/2 0 0];
[ang_q1,vel_q1,acc_q1,t1] = pathPlanJoint(theta0,theta1,0,1);

%  Step 2: The robot drops its manipulator from 𝜃4 = 0 𝑚 𝑡𝑜 𝜃4 =   ― 0.3𝑚. 

theta2 = [pi/2 pi/2 0 -.3];
[ang_q2,vel_q2,acc_q2,t2] = pathPlanJoint(theta1,theta2,0,1);
%  Step 3: The manipulator picks up a 1kg payload and raises it from 𝜃4 = ―0.3𝑚 𝑡𝑜 𝜃4 =  0𝑚. 

theta3 = [pi/2 pi/2 0 0];
[ang_q3,vel_q3,acc_q3,t3] = pathPlanJoint(theta2,theta3,0,1);

plotPathFinal(ang_q1,vel_q1,acc_q1,t1,"Move to Position 1")
plotPathFinal(ang_q2,vel_q2,acc_q2,t2,"Move to Postion 2")
plotPathFinal(ang_q3,vel_q3,acc_q3,t3,"Move to Postion 3")

ang_qT = [ang_q1,ang_q2,ang_q3];
vel_qT = [vel_q1,vel_q2,vel_q3];
acc_qT = [acc_q1,acc_q2,acc_q3];
tt = [ t1,t2,t3];
tt = linspace(0,3,300);
plotPathFinal(ang_qT,vel_qT,acc_qT,tt,"All moves")

% For each step, all joints start and end at 0 velocity and 0 acceleration and take 1s to complete. Using 
% quintic profiles to complete each step in the movement, plot out the complete trajectories of 𝜃, 𝜃, 𝑎𝑛𝑑 
% 𝜃 for each joint. 

%% 

% (4pts) Question 4 Newton Euler Dynamics: 
% Calculate the torque of each joint through the complete movement described in question 3. Model the 
% 1kg payload as a point mass at body frame. Plot the torque of each joint throughout the trajectory. 


samples = length(t1);
tau1 = zeros(numJoints,samples);
% tool_weight =mass(7) - .5* -9.81;
Fsf= [0 0 0] ;

% F1 = T00(1:3,1:3)*(F1 * 9.81)' ;
% F2 = T00(1:3,1:3)*(F2 * 9.81)' ;
% F3 = T00(1:3,1:3)*(F3 * 9.81)' ;
for x = 1:1:samples
   
        tau1(:,x) = motor_torque(M,S,ang_q1(:,x),vel_q1(:,x),acc_q1(:,x),m,Fsf,Ii,G,A);
       
end
tau2 = zeros(numJoints,samples);
for x = 1:1:samples
        tau2(:,x) = motor_torque(M,S,ang_q2(:,x),vel_q2(:,x),acc_q2(:,x),m,Fsf,Ii,G,A);   
end
tau3 = zeros(numJoints,samples);
Fsf= [0 0 9.81] ;
for x = 1:1:samples
   
        tau3(:,x) = motor_torque(M,S,ang_q3(:,x),vel_q3(:,x),acc_q3(:,x),m,Fsf,Ii,G,A);
   
    
end


figure, plot(t1,tau1(1:4,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('Path 1');
legend({'Joint 1','Joint 2','Joint 3','Joint 4'},'Location','southwest','Orientation','horizontal')

figure, plot(t2,tau2(1:4,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('Path 2');
legend({'Joint 1','Joint 2','Joint 3','Joint 4'},'Location','southwest','Orientation','horizontal')

figure, plot(t3,tau3(1:4,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('Path 3');
legend({'Joint 1','Joint 2','Joint 3','Joint 4'},'Location','southwest','Orientation','horizontal')

tauT = [tau1,tau2,tau3];
figure, plot(tt,tauT(1:4,:));
xlabel('time [s]'), ylabel('tau [Nm]'), title('Path 3');
legend({'Joint 1','Joint 2','Joint 3','Joint 4'},'Location','southwest','Orientation','horizontal')




%% Apendex
function AdT = adjoint(T)
% Function creates the 6x6 adjoint transfrom of a homogenous 4x4 transformation matrix
% adjoints can be used to express values in a different referecne frame
%input
%4x4 Transformation matrix

R= T(1:3,1:3);
p = T(1:3,4);

AdT = [ R , zeros(3,3);
        skew(p)*R, R;];
end

function adV = ad(V)
%Function calculates the corresponding 6x6 [𝑎𝑑𝒱1] matrix.
%This matrix can be used to calculate the Lie Bracket of 𝒱1 and another twist 𝒱2 as: [𝑎𝑑𝒱1]𝒱2
%input 
% 1x6 array of a twist 𝒱1 = (𝜔1,𝑣1)
%define values 
w = V(1:3);
v = V(4:6);

adV = [skew(w)     zeros(3,3);
       skew(v)     skew(w);   ];

end
function matrix = skew(w)
%% creates a skew matrix  of 1x3 array of values
matrix = [
    [0 -w(3) w(2)];
    [w(3) 0 -w(1)];
    [-w(2) w(1) 0]];
end
function T = twist2ht(S,theta)
% Given a twist V=Sθ , where S∈R^6 is a Screw Axis and θ is a scalar, function that calculates the corresponding homogeneous transformation matrix T∈SE(3).
% Note: S = [w;v]
%T = e^[S]0
%initalize T
T = eye(4);

%Seperate w and v from s
w = S(1:3);
v = S(4:6);

if norm(w) < eps(10)
    T = [eye(3) v*theta;
        0 0 0 1];
elseif norm(w) - 1 < eps(10)
%Create the Transformation matrix of the twist axis 
R = axisangle2rot(w,theta);
%T(1:3,1:3) = R;
trans = (eye(3)*theta + (1-cos(theta))*skew(w') + (theta-sin(theta))*skew(w')^2)*v;
%T(1:3,4) = trans;
T = [ R trans; 0 0 0 1];
end
end
function R = axisangle2rot(omega,theta)
% Given the exponential coordinates of rotation ω ̂θ, where θ is a scalar and ω∈R^3, with ‖ω‖=1, calculate the corresponding rotation matrix R∈SO(3).
%Rodriquez's formula for screw axis
R = eye(3) + sin(theta)*skew(omega) + (1-cos(theta))*skew(omega)^2;
end
function tau = motor_torque(M,S,q,qd,qdd,m,Ftip,Ii,G,A)


g = 9.81;
numJoints = length(q);
M(3,4,4)= M(3,4,4) + q(4);
%mass per link in kg
%Joint transformations
%Dynamics: Recursive Newton Euler Algorithm (Forward Iteration)
%define twist at frame 0
V0 = zeros(6,1);
Vd0 = [0 0 0 0 0 g]'; % note it is accelerating down
M0 = eye(4,4);
T = zeros(4,4,numJoints);
Vi = zeros(6,numJoints);
Vid = zeros(6,numJoints);
% we are defining the acceration in -g direction
% Ai = [AdM^-1]Si
% A = zeros(6,numJoints);
% for i = 1:numJoints
%     A(:,i) = adjoint(M(:,:,i)^-1)*S(:,i);
% end

for i= 1:numJoints

    if i == 1
       
        T(:,:,i) = twist2ht(A(:,i),q(i))^-1 * M(:,:,i)^-1 * M0;
    
        % Vi = Ai*qdi + [AdTi,i-1]Vi-1
        Vi(:,i) = adjoint(T(:,:,i))*V0 + A(:,i)*qd(i);
    
        % Vdi = Ai*qddi + [Advisor]Ai*Qdi
        Vid(:,i) = adjoint(T(:,:,i)) * Vd0 + ad(Vi(:,i))* A(:,i)*qd(i)+ A(:,i)*qdd(i);

    else

    % Ti,i-1 = e^-[Ai]thetai * Mi^-1 * (Mi-1)
    T(:,:,i) = twist2ht(A(:,i),q(i))^-1 * M(:,:,i)^-1 * M(:,:,i-1);

    % Vi = Ai*qdi + [AdTi,i-1]Vi-1
    Vi(:,i) = adjoint(T(:,:,i))*Vi(:,i-1) + A(:,i)*qd(i);

    % Vdi = Ai*qddi + [Advisor]Ai*Qdi
    Vid(:,i) = adjoint(T(:,:,i)) * Vid(:,i-1) + ad(Vi(:,i))* A(:,i)*qd(i)+ A(:,i)*qdd(i);
    end
    
end


Fi = zeros(6,numJoints+1);
%define the spacial interial matrix
% Ii -> rotational interia matrix 
% mi -> 
% Gi = [Ii 03x3
%       03x3 mi*I3x3]
%Since all modeled points are point masses , Ii = 0
%define the wrench
% Insert the tip into matrix based of 1+ number of joints
%Static Forces
% Rsb = Tsb(1:3,4);
% Rbs = Tbs(1:3, 1:3);
% 
% % FsM = cross(Rsb,Ftip);
% %position1 in kg
% % Fs = [FsM; Ftip];
% Fbf = Rbs * Ftip;
% Fb1 = [0 0 0 Fbf']';

Fb1 = [0 0 0 Ftip]';
%Newtons

Fi(:,numJoints+1) = Fb1;

%calulate the interial matrix
% G = zeros(6,6,numJoints);
tau = zeros(numJoints,1);
% for i = 1:numJoints
%     G(:,:,i) = [Ii(:,:,i)          zeros(3,3);
%       zeros(3,3)  m(i)*eye(3,3);];
% end

%iterate thru backwards
for i = numJoints:-1:1

    if i == numJoints
       
        % Ti+1,i = e^-[Ai+1]thetai+1 * Mi+1^-1 * (Mi)
        T(:,:,i+1) = M(:,:,i)^-1 * M(:,:,i);
    
%     else
%         % Ti+1,i = e^-[Ai+1]thetai+1 * Mi+1^-1 * (Mi)
%         T(:,:,i+1) = twist2ht(A(:,i+1),q(i+1))^-1 * M(:,:,i+1)^-1 * M(:,:,i);
    end
    %calulate the wrench
    % Fi = Gi*Vdi-[adVi]T Gi*Vi + [AdTi+1,i]T Fi+1
    Fi(:,i) = adjoint(T(:,:,i+1))'*Fi(:,i+1) + G(:,:,i) * Vid(:,i) - ad(Vi(:,i))'* (G(:,:,i) * Vi(:,i));

    %taui = Fi^T *Ai
    tau(i) = Fi(:,i)' * A(:,i);
        
end

end