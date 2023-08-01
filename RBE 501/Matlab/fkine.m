function T = fkine(S,M,q)
% Write a function `fkine' to calculate the forward kinematics of a robotic arm using the Product of Exponentials Formula. The function takes as input a matrix S containing all the screw axes (each column is a screw axis, with column 1 being the screw axis for the first joint, column 2 being the screw axis for the second joint, and so on), a vector of joint variables q, and an homogeneous transformation matrix M representing the robot pose in its home configuration.
% Note the screw axis should contain omega in the first 3 rows and velocity in rows 4:6. For example S(:,i) = [w;v]
% initialize values
n = length(q);
T=eye(4);

%loop each row of S and multiple the twist axis transformation for each axis of value theta
for i = 1: n
   T =T*twist2ht(S(:,i),q(i));
   
end

T = T * M;

end

function J = jacob0(S,q)
%Write a function `jacob0' to calculate the Jacobian matrix for a robotic arm. The Jacobian must 
%be calculated in the space frame. The function takes as input a matrix S containing all the screw 
%axes (arranged by column) and a vector of joint variables q. The function should be able to 
%calculate the Jacobian for kinematic chains with any number `n' of joints.
%(Hint: Consider creating a dictionary or list of Transformation matrices which can then help you 
%find the Jacobians)

%Js = [ws; vs]

%Vs = Js0 *0dot

%Vs = [Js1 Js2 ...]

%Vs = [V1 V2 V3 ....]

%Adj = e^s101 [s2] e^-s101

T=eye(4);

%loop each row of S and multiple the twist axis transformation for each axis of value theta
%Js1 = I* S1
%Js2 = T01 * S2
%Js3 = T02 * S3 
%Js4 = .......
for i = 1:size(S)
   J(1:6,i) = adjoint(S(:,i),T)';
   Tn = twist2ht(S(:,i),q(1:i));
   T = T * Tn;
   
end


end


function Vtrans = adjoint(V,T)
%Given a twist 𝒱  expressed with respect to some arbitrary reference frame, calculate its 
%representation in a new frame, whose position and orientation is described by a homogeneous 
%transformation matrix T
% Va = AdjTab * Vb

R = T(1:3,1:3);
p = T(1:3,4);
%T = [ R   P
%      0   1]

% AdjT = [    R      O3x3
%           [P]R       R ]
AdjT = [ R , zeros(3,3);
        skew(p')*R, R;];
% AdjT(1:3,1:3) = R;
% AdjT(4:6,1:3) = skew(p')*R;
% AdjT(1:3,4:6) = zeros(3);
% AdjT(4:6,4:6) = R;

Vtrans = AdjT * V;


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

function matrix = skew(w)
%% creates a skew matrix  of 1x3 angular velocity w
matrix = [
    [0 -w(3) w(2)];
    [w(3) 0 -w(1)];
    [-w(2) w(1) 0]];
end


function R = axisangle2rot(omega,theta)
% Given the exponential coordinates of rotation ω ̂θ, where θ is a scalar and ω∈R^3, with ‖ω‖=1, calculate the corresponding rotation matrix R∈SO(3).
%Rodriquez's formula for screw axis
R = eye(3) + sin(theta)*skew(omega) + (1-cos(theta))*skew(omega)^2;
end

