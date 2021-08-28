clear all real
close all
clc

% a = [1 2 3 4 5];
% b = [1 3 2 5 8];
% 
% fig1 = figure;
% ax1 = uiaxes(fig1);
% fig2 = figure;
% ax2 = uiaxes(fig2);
% 
% %hlink = linkprop([ax1,ax2],{'Children'});
% 
% h1 = plot(ax1,a,b);
% hold(ax1);
% h2 = plot(ax1,b,a);
% 
% % h3 = plot(ax2,a,b);
% % hold(ax2)
% % h4 = plot(ax2,b,a);
% 
% copyobj(ax1.Children, ax2)
% 
% %ax.Children(1).XData = [2 2 1 1 1 1 1 1 7];
% h1.XDataSource = 'a';
% h1.YDataSource = 'b';
% % h3.XDataSource = 'a';
% % h3.YDataSource = 'b';
% 
% refreshdata([ax1,ax2]);
% 
% 
% [X,Y,Z] = peaks;
% surf(X,Y,Z)
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

% ax = uiaxes;
% dummyS.faces = [1,2,3];
% dummyS.vertices = [0,0,0;0,0,0;0,0,0];
% p = patch(ax,dummyS,'FaceColor','b','EdgeColor','none');
% 
% a = aircraft;
% graphData.Position = [0 150 0; 5 150 5; 10 160 10; 15 140 15; 20 150 20];
% moveAircraft(a,graphData.Position(1,:));
% p.Vertices = a.outPatchData.vertices;
% p.Faces = a.outPatchData.faces;
% refreshdata(app.TridViewUIAxes);
% drawnow;

ax = uiaxes;
a = aircraft;
hold(ax);

fv = stlread('Extra 300.stl');
%fv2D = fv;
%fv2D.vertices = fv2D.vertices(:,2:3);

%plot3(ax,[2 3],[4 7],[300 200])
%patch(ax,'XData',[-350,-350,350,350],'YData',[-10,150,150,-10],'ZData',[150,150,150,150],'FaceColor','b');

plot3(ax,[0.5 0.2],[0.67 -1],[-0.4 100],'-o')
%plot(ax,[0.5 0.2],[0.67 -1],'-o');
patch(ax,a.outPatchData,'EdgeColor','none');
%patch(ax,fv2D,'EdgeColor','none');

%ax.Interactions = regionZoomInteraction;
view(ax,0,0)
ax.DataAspectRatio = [1 1 1];
ax.XLim = [-100,100];
ax.YLim = [-100,100];
ax.ZLim = [-100,100];

ax.ClippingStyle = 'rectangle';

%ax.Interactions = regionZoomInteraction;


 