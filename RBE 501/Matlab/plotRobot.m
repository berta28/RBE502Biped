function p1 =plotRobot(S,M,q,st)
numJoints = length(q);
T = zeros(4,4,numJoints);
plotFrame([1 0 0 0;0 1 0 0; 0 0 1 0;0 0 0 1],50/1000, 3,1);

for idx = 1:numJoints
    T(:,:,idx) = double(fkine(S(:,1:idx),M(:,:,idx),q(1:idx)));
%     display(T(:,:,idx));
    plotFrame(T(:,:,idx),50/1000, 3,1);
end

p1 = plotLink(T, 1,st);

axis equal
xlim([-600 400]/1000)
ylim([-300 300]/1000)
zlim([0 500]/1000)

hold off
end

