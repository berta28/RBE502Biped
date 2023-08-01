function [q,qdiff,qddiff,t] = pathPlanJoint(q0,qf,t0,tf)
% t0=0;
% tf=1;
qd0 = 0;
qdf = 0;
qdd0= 0;
qddf = 0;
numSamples = 10;
t = linspace(t0,tf,numSamples);


numJoints =length(q0);

q= zeros(numJoints,numSamples);
qdiff= zeros(numJoints,numSamples);
qddiff= zeros(numJoints,numSamples);
for x = 1:numJoints
   
    
    A = quinticpoly(t0,tf,q0(x),qf(x),qd0,qdf,qdd0,qddf);

    q(x,:)      = A(1) + A(2) .* t +   A(3) .* t.^2 +   A(4) .* t.^3 +    A(5) .* t.^4 +    A(6) .* t.^5;
    qdiff(x,:)  =        A(2)      + 2*A(3) .* t    + 3*A(4) .* t.^2 +  4*A(5) .* t.^3 +  5*A(6) .* t.^4;
    qddiff(x,:) =                    2*A(3)         + 6*A(4) .* t    + 12*A(5) .* t.^2 + 20*A(6) .* t.^3;

    
end
end