function frames = animatePath(Td,jointAngles,S,M)

fig = figure('Name','Animated Path');
p = uipanel(fig,"Position",[0.1 0.1 0.8 0.8],...
    "BackgroundColor","w");
axes(p);
fig.Visible = 'off';
loops = size(jointAngles,2)/20;
frames(loops) = struct('cdata',[],'colormap',[]);
frameCnt = 0;
% plot start position
plotFrame(Td,50,3,0.3)
q = jointAngles(:,1);
plotRobot(S,M,q)
refreshdata
drawnow
frameCnt = frameCnt + 1;
frames(frameCnt) = getframe(gcf);
hold off

skipframe = 20;
for time_idx = skipframe:skipframe:size(jointAngles,2)
    plotFrame(Td,50,3,0.3)
    q = jointAngles(:,time_idx);
    plotRobot(S,M,q)
    refreshdata
    drawnow
    frameCnt = frameCnt + 1;
    frames(frameCnt) = getframe(gcf);
    hold off
    time_idx
end
% fig.Visible = 'on';
%     x = linspace(-1, 1, 75);
%     y = -x.^2;
%     fig = figure('CloseRequestFcn', @closereq);
%     ax = axes(fig);
%     while true
%     for k2 = 1:2
%         for k1 = 1:length(x)
%             plot(ax, x(k1)*(-1)^k2, y(k1), 'ob', 'MarkerFaceColor','r')
%             axis([min(x)  max(x)    min(y)  max(y)])
%             refreshdata
%             drawnow
%         end
%     end
%     end
end
