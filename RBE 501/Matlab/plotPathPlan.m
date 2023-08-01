function plotPathFinal(q,qdiff,qddiff,t,name)


% tiledlayout(3,1)
figure, subplot(3,1,1);
plot(t,q(1,:))
xlabel('time [s]'), ylabel('q1 [m]'), title(name+' Position  Joint 1');
legend({'Joint 1'},'Location','northwest','Orientation','horizontal');

figure, subplot(3,1,2), plot(t,qdiff(1,:));
xlabel('time [s]'), ylabel('qdot [m/s]'), title(name+' Velocity Joint 1');
legend({'Joint 1'},'Location','northwest','Orientation','horizontal');

figure, subplot(3,1,3), plot(t,qddiff(1,:));
xlabel('time [s]'), ylabel('qddot [m/s^2]'), title(name+ ' Acceleration Joint 1');
legend({'Joint 1'},'Location','northwest','Orientation','horizontal');

figure, plot(t,q(2:6,:));
xlabel('time [s]'), ylabel('q [rad]'), title(name+ ' Position');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'},'Location','northwest','Orientation','horizontal')

figure, plot(t,qdiff(2:6,:));
xlabel('time [s]'), ylabel('qdot [rad/s]'), title(name+ ' Velocity');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'},'Location','northwest','Orientation','horizontal')

figure, plot(t,qddiff(2:6,:));
xlabel('time [s]'), ylabel('qddot [rad/s^2]'), title(name+ ' Acceleration');
legend({'Joint 2','Joint 3','Joint 4','Joint 5','Joint 6'},'Location','northwest','Orientation','horizontal')