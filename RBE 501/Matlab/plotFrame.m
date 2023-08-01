function [] = plotFrame(T,scaling,linewidth,transparency)

x = T(1:3,1);
y = T(1:3,2);
z = T(1:3,3);

framePosition = T(1:3,4);

x_end = framePosition + scaling * x;
y_end = framePosition + scaling * y;
z_end = framePosition + scaling * z;

plot3([framePosition(1) x_end(1)],...
      [framePosition(2) x_end(2)],...
      [framePosition(3) x_end(3)],...
      'Color',[1 0 0 transparency],'LineWidth',linewidth);
hold on;

plot3([framePosition(1) y_end(1)],...
      [framePosition(2) y_end(2)],...
      [framePosition(3) y_end(3)],...
      'Color',[0 1 0 transparency],'LineWidth',linewidth);
hold on;
plot3([framePosition(1) z_end(1)],...
      [framePosition(2) z_end(2)],...
      [framePosition(3) z_end(3)],...
      'Color',[0 0 1 transparency],'LineWidth',linewidth);
hold on;
end

