clear all real
close all
clc

x = [200,-600,400];
y = [-600,400,100];
z = [-150,100,350];

fv = stlread('.\AircraftModels\Gemini.stl');

ax = uiaxes;

p = patch(ax,fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
xlabel(ax,'X');
ylabel(ax,'Y');
zlabel(ax,'Z');
ax.DataAspectRatio = [1 1 1];

ax.Children(1).Vertices = rotatepoint(randrot,fv.vertices);

% % % Add a camera light, and tone down the specular highlighting
% camlight(ax,'headlight');
% material(p,'dull');
% % 
% % % Fix the axes scaling, and set a nice view angle
% axis(ax,'image');
% view(ax,[-135 35]);
     
% hold(ax)
% plot3(ax,x,y,z,'-o');

% dummyS.faces = [1,2,3];
% dummyS.vertices = [0,0,0;0,0,0;0,0,0];
% 
% p = patch(dummyS);
