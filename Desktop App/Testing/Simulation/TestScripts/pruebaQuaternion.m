f = figure;
ax = axes(f);
hold(ax,'on')
p = [0,0,0];
lenArrow = 5; %length of the body local frame axis
pBv = eye(3)*lenArrow;
quat = quaternion(0.924,0.383,0,0);
pBvRot = rotateframe(quat,pBv);
% Original frame
quiver3(ax,p(1),p(2),p(3),pBv(1,1),pBv(1,2),pBv(1,3),'Color','r'); %x axis
quiver3(ax,p(1),p(2),p(3),pBv(2,1),pBv(2,2),pBv(2,3),'Color','g'); %y axis
quiver3(ax,p(1),p(2),p(3),pBv(3,1),pBv(3,2),pBv(3,3),'Color','b'); %z axis
%Rotated frame
quiver3(ax,p(1),p(2),p(3),pBvRot(1,1),pBvRot(1,2),pBvRot(1,3),'Color','r','LineStyle','--'); %x axis
quiver3(ax,p(1),p(2),p(3),pBvRot(2,1),pBvRot(2,2),pBvRot(2,3),'Color','g','LineStyle','--'); %y axis
quiver3(ax,p(1),p(2),p(3),pBvRot(3,1),pBvRot(3,2),pBvRot(3,3),'Color','b','LineStyle','--'); %z axis

hold(ax,'off')
ax.XLabel.String = 'x';
ax.YLabel.String = 'y';
ax.ZLabel.String = 'z';
legend('X','Y','Z','Xrot','Yrot','Zrot');
ax.XLim = [-10,10];
ax.YLim = [-10,10];
ax.ZLim = [-10,10];
ax.DataAspectRatio = [1 1 1];
view(ax,30,20);
grid(ax,'on')