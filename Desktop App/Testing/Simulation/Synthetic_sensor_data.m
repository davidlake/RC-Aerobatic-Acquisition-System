%% Axes and environment preparation
f = uifigure;
ax = axes(f);
drawFlightTrack(ax);
%% Ideal path generation
% Waypoints = [-260,150, -200;...
%             260,150, -200];
Waypoints = [-260,150, -200;...
            -150,170, -200;...
            0,150, -200;...
            150,130, -200;...
            260,150, -200];
% Waypoints = [-260,150, -50;...
%             -150,150, -100;...
%             0,150, -100;...
%             150,150, -200;...
%             260,150, -200];
% Waypoints = [-260,150, -100;...
%             200,150, -100;...
%             250,150, -150;...
%             200,150, -200];
meanSpeed = 150; %km/h
[totalDist,TimeOfArrival] = calcDistAndIntervals(Waypoints,meanSpeed);
fs = 100; %sample rate in Hz
trajectory = waypointTrajectory(Waypoints,TimeOfArrival,'ReferenceFrame','NED','SampleRate',fs,'AutoBank',true,'AutoPitch',true);
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
lenArrow = 30; %length of the body local frame axis
NED_axis = [1 0 0; 0 1 0; 0 0 1]*lenArrow;
bodyFrameAxis = zeros(3,3,nElements); % aircraft body frame (NED)
for c=1:nElements
    bodyFrameAxis(:,:,c) = rotatepoint(trueOrientation(c),NED_axis);
end
%% Simulate altimeter sensor data
% fsAltimeter = 50;
% altimeter = altimeterSensor('SampleRate',fsAltimeter,...
%                             'ReferenceFrame','NED',...
%                             'ConstantBias',0.01,...
%                             'NoiseDensity',0.05,...
%                             'BiasInstability',0.05,...
%                             'DecayFactor',0.5);
% trajectory.SampleRate = fsAltimeter;
% reset(trajectory);
% nElementsAltimeter = round(trajectory.TimeOfArrival(end)*trajectory.SampleRate)-1;
% truePositionAltimeter = zeros(nElementsAltimeter,3); 
% c = 1;
% while ~isDone(trajectory)
%    [truePositionAltimeter(c,:),~,~,~,~] = trajectory();
%    c = c + 1;
% end
% altimeter_PositionZ = altimeter(truePositionAltimeter);
%% Simulate GPS receiver data
% fsGPS = 5;
% refLoc = [43.2998346, -8.4301743, 260];
% GPS = gpsSensor('SampleRate',fsGPS,...
%                 'ReferenceFrame','NED','ReferenceLocation',refLoc,...
%                 'HorizontalPositionAccuracy',1.6,'VerticalPositionAccuracy',3,'VelocityAccuracy',0.1,...
%                 'DecayFactor',0.5);
% trajectory.SampleRate = fsGPS;
% reset(trajectory);
% nElementsGPS = round(trajectory.TimeOfArrival(end)*trajectory.SampleRate)-1;
% truePositionGPS = zeros(nElementsGPS,3); 
% trueVelocityGPS = zeros(nElementsGPS,3);
% c = 1;
% while ~isDone(trajectory)
%    [truePositionGPS(c,:),~,trueVelocityGPS(c,:),~,~] = trajectory();
%    c = c + 1;
% end
% [positionLLA,velocity,groundspeed,course] = GPS(truePositionGPS,trueVelocityGPS);
% gps_Position = lla2flat(positionLLA,refLoc(1:2),0,-refLoc(3));
% % [gps_Position2(:,1),gps_Position2(:,2),gps_Position2(:,3)] = latlon2local(positionLLA(:,1),positionLLA(:,2),positionLLA(:,3),refLoc);
%% Simulate IMU sensor data
fsIMU = 100;
accPar = accelparams;
% accPar = accelparams('MeasurementRange',(4*9.81),'Resolution',0.0012, ...
%                      'ConstantBias', 0,...%0.4905,...
%                      'AxesMisalignment',0,...
%                      'NoiseDensity',0.0023,...
%                      'BiasInstability', 0.05,...%
%                      'RandomWalk',0.2,...
%                      'TemperatureBias',0.0102,'TemperatureScaleFactor',0.026);
gyroPar = gyroparams;
% gyroPar = gyroparams('MeasurementRange',17.4533,'Resolution',5.3211e-04, ...
%                      'ConstantBias',0,...%0.0873,...
%                      'AxesMisalignment',0,...
%                      'NoiseDensity',2.6180e-04,...
%                      'BiasInstability',0.05,...%
%                      'RandomWalk',0.05,...
%                      'TemperatureBias',8.7266e-04,'TemperatureScaleFactor',0,...%,0.5236,...
%                      'AccelerationBias',0);
magPar = magparams;
% magPar = magparams('MeasurementRange',4900,'Resolution',0.15, ...
%                    'ConstantBias',0,...
%                    'AxesMisalignment',0,...
%                    'NoiseDensity',0,...
%                    'BiasInstability',0,...
%                    'RandomWalk',0,...
%                    'TemperatureBias',0,'TemperatureScaleFactor',0);
IMU = imuSensor('accel-gyro-mag', 'ReferenceFrame', 'NED', 'SampleRate', fsIMU,...
                'Temperature', 25,...
                'MagneticField', [24.11475 -0.7018 38.9986],...
                'Accelerometer',accPar,'Gyroscope',gyroPar,'Magnetometer',magPar);
trajectory.SampleRate = fsIMU;
reset(trajectory);
nElementsIMU = round(trajectory.TimeOfArrival(end)*trajectory.SampleRate)-1;
trueAccelerationIMU = zeros(nElementsIMU,3);
trueAngularVelocityIMU = zeros(nElementsIMU,3); 
trueOrientationIMU = zeros(nElementsIMU,1,'quaternion');
c = 1;
while ~isDone(trajectory)
   [~,trueOrientationIMU(c),~,trueAccelerationIMU(c,:),trueAngularVelocityIMU(c,:)] = trajectory();
   c = c + 1;
end
[IMU_acceleration,IMU_angularVel,IMU_magneticF] = IMU(trueAccelerationIMU,trueAngularVelocityIMU,trueOrientationIMU);
%% Signal readings plots
g = figure;
ax11 = subplot(3,3,1);
plot(ax11,trueAccelerationIMU(:,1));
hold on
plot(ax11,IMU_acceleration(:,1));
ax12 = subplot(3,3,4);
plot(ax12,trueAccelerationIMU(:,2));
hold on
plot(ax12,IMU_acceleration(:,2));
ax13 = subplot(3,3,7);
plot(ax13,trueAccelerationIMU(:,3));
hold on
plot(ax13,IMU_acceleration(:,3));

ax21 = subplot(3,3,2);
plot(ax21,trueAngularVelocityIMU(:,1));
hold on
plot(ax21,IMU_angularVel(:,1));
ax22 = subplot(3,3,5);
plot(ax22,trueAngularVelocityIMU(:,2));
hold on
plot(ax22,IMU_angularVel(:,2));
ax23 = subplot(3,3,8);
plot(ax23,trueAngularVelocityIMU(:,3));
hold on
plot(ax23,IMU_angularVel(:,3));

ax31 = subplot(3,3,3);
% plot(ax31,trueAngularVelocityIMU(:,1));
% hold on
plot(ax31,IMU_magneticF(:,1));
ax32 = subplot(3,3,6);
% plot(ax32,trueAngularVelocityIMU(:,2));
% hold on
plot(ax32,IMU_magneticF(:,2));
ax33 = subplot(3,3,9);
% plot(ax33,trueAngularVelocityIMU(:,3));
% hold on
plot(ax33,IMU_magneticF(:,3));

%% Plots
hold(ax,'on');
%Ideal path
plot3(ax,Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'LineStyle','none','Marker','.','MarkerEdgeColor','k');
plot3(ax,truePosition(:,1),truePosition(:,2),truePosition(:,3),'Color','k');
%GPS path
%plot3(ax,position(:,1),position(:,2),position(:,3),'LineStyle','none','Marker','x','MarkerEdgeColor','r');

%Step by step simulation orientation
qBFx = quiver3(ax,0,0,0,0,0,0,'Color','r','LineWidth',1.5,'MaxHeadSize',1);
qBFy = quiver3(ax,0,0,0,0,0,0,'Color','g','LineWidth',1.5,'MaxHeadSize',1);
qBFz = quiver3(ax,0,0,0,0,0,0,'Color','b','LineWidth',1.5,'MaxHeadSize',1);


FPS = 25; %desired FPS 
ratePlotDiv = round(fs/FPS); % reduces the FPS to plot
r = rateControl(fs/ratePlotDiv); % fs/ratePlotDiv = FPS (but a round is needes if division is not exact)
for c=0:ratePlotDiv:nElements
    if (c == 0)
        c=1;
    end
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
   waitfor(r);
end
%stats = statistics(r)


