% change Js to jacobian values for your robot w.r.t. space frame
Js = [1 1 1 1 1 1];

F1 = [0 0 0 0.961 -0.961 0]';
Tau1 = sTorque(Js, F1)
F2 = [0 0 0 -0.961 -0.961 0]';
Tau2 = sTorque(Js, F2)
F3 = [0 0 0 0 -1.36 0]';
Tau3 = sTorque(Js, F3)

% calculates torque w.r.t. space frame
function t = sTorque(Js, Fs)
    t = Js'.*Fs;
end
