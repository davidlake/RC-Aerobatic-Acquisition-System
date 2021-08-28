%% Axes and environment preparation
f = uifigure;
ax = axes(f);
%[~] = drawFlightTrack(ax);
ax.DataAspectRatio = [1 1 1];
ax.XDir = 'reverse';
ax.ZDir = 'reverse';
ax.XLabel.String = 'x North';
ax.YLabel.String = 'y East';
ax.ZLabel.String = 'z Down';
grid(ax,'on');
view(ax,30,20);
%% Ideal path generation
Waypoints = [-300,100,0;...
            100,100,0;...
            200,100,0;...
            300,200,0;...
            200,300,0;...
            100,300,0;...
            -300,300,0];
meanSpeed = 100; %km/h
[totalDist,timeIntervals] = calcDistAndIntervals(Waypoints,meanSpeed);
meanSpeed = meanSpeed*1000/3600;
TimeOfArrival = [0; cumsum(timeIntervals)];
cumsum(timeIntervals)
Velocities = [meanSpeed,0,0;...
            meanSpeed,0,0;...
            meanSpeed,0,0;...
            0,meanSpeed,0;...
            -meanSpeed,0,0;...
            -meanSpeed,0,0;...
            -meanSpeed,0,0];        
Orientation = [0,0,0;...
            0,0,0;...
            0,0,0;...
            0,-90,0;...
            0,-180,0;...
            0,-180,0;...
            0,-180,0];  
qOrientation = quaternion(Orientation,'eulerd','XYZ','frame');
fs = 10;
trajectory = waypointTrajectory(Waypoints,TimeOfArrival,'SampleRate',fs,'SamplesPerFrame',1,'Velocities',Velocities,'Orientation',qOrientation);
nElements = round(trajectory.TimeOfArrival(end)*trajectory.SampleRate)-1;
truePosition = zeros(nElements,3); 
trueOrientation = zeros(nElements,1,'quaternion');
trueVelocity = zeros(nElements,3);
trueAcceleration = zeros(nElements,3);
trueAngularVelocity = zeros(nElements,3); 
c = 1;
while ~isDone(trajectory)
   [truePosition(c,:),trueOrientation(c),trueVelocity(c,:),trueAcceleration(c,:),trueAngularVelocity(c,:)] = trajectory();
   c = c + 1;
end
%Create body frame axis value for each instant
bodyFrameAxis = zeros(3,3,nElements);
lenArrow = 30; %length of the body local frame axis
nedAxis = eye(3)*lenArrow;
for c=1:nElements
    bodyFrameAxis(:,:,c) = rotateframe(trueOrientation(c),nedAxis);
end

truePosition = rotateframe(quaternion([90,0,0],'eulerd','XYZ','frame'),truePosition);
%rotateframe(quaternion([90,0,0],'eulerd','XYZ','frame'),trueOrientation)

vPoint = vecnorm(trueVelocity,2,2)*3600/1000; %velocity norm vector
%% Simulated sensor data generation
%GPS
refLoc = [43.2998346, -8.4301743, 260];
fsGPS = 5;
GPS = gpsSensor('SampleRate',fsGPS,...
                'ReferenceFrame','NED','ReferenceLocation',refLoc,...
                'HorizontalPositionAccuracy',1.6,'VerticalPositionAccuracy',3,'VelocityAccuracy',0.1,...
                'DecayFactor',0.5);
trajectory.SampleRate = fsGPS;
reset(trajectory);
nElementsGPS = round(trajectory.TimeOfArrival(end)*trajectory.SampleRate)-1;
truePositionGPS = zeros(nElementsGPS,3); 
trueVelocityGPS = zeros(nElementsGPS,3);
c = 1;
while ~isDone(trajectory)
   [truePositionGPS(c,:),~,trueVelocityGPS(c,:),~,~] = trajectory();
   c = c + 1;
end
[positionLLA,velocity,groundspeed,course] = GPS(truePositionGPS,trueVelocityGPS);
position = lla2flat(positionLLA,refLoc(1:2),0,-refLoc(3));
%% Plots
hold(ax,'on');
%Ideal path
plot3(ax,Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'LineStyle','none','Marker','.','MarkerEdgeColor','k');
plot3(ax,truePosition(:,1),truePosition(:,2),truePosition(:,3),'Color','k');
%GPS path
%plot3(ax,position(:,1),position(:,2),position(:,3),'LineStyle','none','Marker','x','MarkerEdgeColor','r');

%Step by step simulation orientation
qBFx = quiver3(ax,0,0,0,0,0,0,'Color','r');
qBFy = quiver3(ax,0,0,0,0,0,0,'Color','g');
qBFz = quiver3(ax,0,0,0,0,0,0,'Color','b');
tic
for c=1:nElements
   %plot3(ax,truePosition(c,1),truePosition(c,2), truePosition(c,3), 'bo')
   qBFx.XData = truePosition(c,1);
   qBFx.YData = truePosition(c,2); 
   qBFx.ZData = truePosition(c,3); 
   qBFx.UData = bodyFrameAxis(1,1,c);
   qBFx.VData = bodyFrameAxis(1,2,c);
   qBFx.WData = bodyFrameAxis(1,3,c); 
   qBFy.XData = truePosition(c,1);
   qBFy.YData = truePosition(c,2); 
   qBFy.ZData = truePosition(c,3);
   qBFy.UData = bodyFrameAxis(2,1,c);
   qBFy.VData = bodyFrameAxis(2,2,c);
   qBFy.WData = bodyFrameAxis(2,3,c);
   qBFz.XData = truePosition(c,1);
   qBFz.YData = truePosition(c,2); 
   qBFz.ZData = truePosition(c,3);
   qBFz.UData = bodyFrameAxis(3,1,c);
   qBFz.VData = bodyFrameAxis(3,2,c);
   qBFz.WData = bodyFrameAxis(3,3,c);
%    quiver3(ax,truePosition(c,1),truePosition(c,2),truePosition(c,3),bodyFrameAxis(1,1,c),bodyFrameAxis(1,2,c),bodyFrameAxis(1,3,c),'Color','r'); %x axis
%    quiver3(ax,truePosition(c,1),truePosition(c,2),truePosition(c,3),bodyFrameAxis(2,1,c),bodyFrameAxis(2,2,c),bodyFrameAxis(2,3,c),'Color','g'); %y axis
%    quiver3(ax,truePosition(c,1),truePosition(c,2),truePosition(c,3),bodyFrameAxis(3,1,c),bodyFrameAxis(3,2,c),bodyFrameAxis(3,3,c),'Color','b'); %z axis
   pause(1/fs);
end
toc
hold(ax,'off');

% hold(ax,'on');


