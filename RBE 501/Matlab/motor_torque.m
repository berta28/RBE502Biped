function tau = motor_torque(M,S,q,qd,qdd,m,Ftip)


g = 9.81;
numJoints = length(q);
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
A = zeros(6,numJoints);
for i = 1:numJoints
    A(:,i) = adjoint(M(:,:,i)^-1)*S(:,i);
end

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
Ii= zeros(3,3,6);
%define the wrench
% Insert the tip into matrix based of 1+ number of joints
%Static Forces
Rsb = Tsb(1:3,4);
Rbs = Tbs(1:3, 1:3);

% FsM = cross(Rsb,Ftip);
%position1 in kg
% Fs = [FsM; Ftip];
Fbf = Rbs * Ftip;
Fb1 = [0 0 0 Fbf']';
%Newtons

Fi(:,numJoints+1) = Fb1;

%calulate the interial matrix
G = zeros(6,6,numJoints);
tau = zeros(numJoints,1);
for i = 1:numJoints
    G(:,:,i) = [Ii(:,:,i)          zeros(3,3);
      zeros(3,3)  m(i)*eye(3,3);];
end

%iterate thru backwards
for i = numJoints:-1:1

    if i == numJoints
       
        % Ti+1,i = e^-[Ai+1]thetai+1 * Mi+1^-1 * (Mi)
        T(:,:,i+1) = M(:,:,i+1)^-1 * M(:,:,i);
    
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