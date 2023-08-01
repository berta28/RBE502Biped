function plotPathPlan(q,qdiff,qddiff,t,name)

figure(Position=[2,2,800,800])
subplot(3,1,1),plot(t,q(1:4,:));
xlabel('time [s]'), ylabel('q [rad]'), title(name+ ' Position');
legend({'Joint 1','Joint 2','Joint 3',},'Location','northwest','Orientation','horizontal')

subplot(3,1,2), plot(t,qdiff(1:4,:));
xlabel('time [s]'), ylabel('qdot [rad/s]'), title(name+ ' Velocity');
legend({'Joint 1','Joint 2','Joint 3',},'Location','northwest','Orientation','horizontal')

subplot(3,1,3), plot(t,qddiff(1:4,:));
xlabel('time [s]'), ylabel('qddot [rad/s^2]'), title(name+ ' Acceleration');
legend({'Joint 1','Joint 2','Joint 3',},'Location','northwest','Orientation','horizontal')
end