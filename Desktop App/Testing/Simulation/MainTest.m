close all
%% Axes and environment preparation
f = uifigure;
ax = axes(f);
drawFlightTrack(ax);
%% Ideal path generation
% trajectory = waypointTrajectory( ...
%     [-260 150 -150;-110 130 -150;110 170 -150;260 151 -150], ...
%     [0;3.61458875301293;8.94774435993623;12.5589771688403], ...
%     'Course', [-12.7163172522558;2.65810786728477;2.91028297342141;-12.2791406504979], ...
%     'GroundSpeed', [42;42;42;42], ...
%     'ClimbRate', [0;0;0;0], ...
%     'AutoPitch', true, ...
%     'AutoBank', true);
trajectory = waypointTrajectory( ...
    [-262 149 -150;-110 130 -150;119 170 -150;260 151 -150;258 89 -160;-3 129 -203;-176 175 -183;-262 92 -193;-45 79 -256;189 114 -300], ...
    [0;3.65156742344469;9.71520743674713;15.3463149590628;18.5850354070265;28.2438884541207;34.141208722865;40.7543740107749;50.1003297397989;55.702335627459], ...
    'Course', [-10.2537600168192;-0.865393676349325;17.1553568481255;-61.1921865657139;-119.26782113679;147.758792988754;-171.846262729174;-84.1364696233199;22.8271761995809;1.3594384166101], ...
    'GroundSpeed', [42;42;35;20;20;42;20;20;35;50], ...
    'ClimbRate', [0;0;0;1;4;0;-3;4;10;0], ...
    'AutoPitch', true, ...
    'AutoBank', true);
% trajectory = waypointTrajectory( ...
%     [0 150 -150;2000 150 -150], ...
%     [0;47.9961603071754], ...
%     'Course', [0;0], ...
%     'GroundSpeed', [41.67;41.67], ...
%     'ClimbRate', [0;0], ...
%     'AutoPitch', true, ...
%     'AutoBank', true);
% Waypoints = [-260,150, -200;...
%             260,150, -200];
% Waypoints = [-260,150, -200;...
%             -150,170, -200;...
%             0,150, -200;...
%             150,130, -200;...
%             260,150, -200];
% Waypoints = [-260,150, -50;...
%             -150,150, -100;...
%             0,150, -100;...
%             150,150, -200;...
%             260,150, -200];
% Waypoints = [-260,150, -100;...
%             200,150, -100;...
%             250,150, -150;...
%             200,150, -200];
% meanSpeed = 150; %km/h
% [totalDist,TimeOfArrival] = calcDistAndIntervals(Waypoints,meanSpeed);
fs = 100; %sample rate in Hz
% trajectory = waypointTrajectory(Waypoints,TimeOfArrival,'ReferenceFrame','NED','SampleRate',fs,'AutoBank',true,'AutoPitch',true);
[truePos,trueOri,trueVel,trueAcc,trueAngV] = lookupPose(trajectory, trajectory.TimeOfArrival(1):1/trajectory.SampleRate:trajectory.TimeOfArrival(end));
nElements = length(truePos(:,1));
tTrue = 0:1/trajectory.SampleRate:1/trajectory.SampleRate*(nElements-1);
lenArrow = 30; %length of the body local frame axis
NED_axis = [1 0 0; 0 1 0; 0 0 1]*lenArrow;
bodyFrameAxisT = zeros(3,3,nElements); % aircraft body frame (NED)
for c=1:nElements
    bodyFrameAxisT(:,:,c) = rotatepoint(trueOri(c),NED_axis);
end
%% Plot theoretical trajectory vectors
% figure
% subplot(3,2,[1,3])
% plot(tTrue,truePos);
% title('Posición')
% xlabel('Tiempo [s]')
% ylabel('Posición [m]')
% legend({'x','y','z'})
% [yawTrue, pitchTrue, rollTrue] = quat2angle(compact(trueOri));
% subplot(3,2,5)
% plot(tTrue,rad2deg(yawTrue));
% hold on
% plot(tTrue,rad2deg(pitchTrue));
% plot(tTrue,rad2deg(rollTrue));
% title('Orientación')
% xlabel('Tiempo [s]')
% ylabel('Orientación [deg]')
% legend({'yaw','pitch','roll'})
% subplot(3,2,2)
% plot(tTrue,trueVel);
% title('Velocidad lineal')
% xlabel('Tiempo [s]')
% ylabel('Velocidad [m/s]')
% legend({'x','y','z'})
% subplot(3,2,4)
% plot(tTrue,trueAcc);
% title('Aceleración lineal')
% xlabel('Tiempo [s]')
% ylabel('Velocidad [m/s2]')
% legend({'x','y','z'})
% subplot(3,2,6)
% plot(tTrue,trueAngV);
% title('Velocidad angular')
% xlabel('Tiempo [s]')
% ylabel('Velocidad [rad/s]')
% legend({'x','y','z'})
%% Generate syntethic data ALTIMETER
fsALT = 50;
%Real
altimeter = altimeterSensor('SampleRate',fsALT,...
                            'ReferenceFrame','NED',...
                            'ConstantBias',0.01,...
                            'NoiseDensity',0.05,...
                            'BiasInstability',0.05,...
                            'DecayFactor',0.5);  
ALT_alt = -altimeter(truePos(1:fs/fsALT:end,:));
tALT = 0:1/fsALT:1/fsALT*(length(ALT_alt)-1);
%Ideal
% altimeterI = altimeterSensor('SampleRate',fsALT,...
%                             'ReferenceFrame','NED'); 
% ALT_altI = -altimeterI(truePos(1:fs/fsALT:end,:));
%% Generate syntethic data GPS RECEIVER
fsGPS = 5;
refLoc = [43.2998346, -8.4301743, 200];
%Real
GPS = gpsSensor('SampleRate',fsGPS,...
                'ReferenceFrame','NED','ReferenceLocation',refLoc,...
                'HorizontalPositionAccuracy',1.6,'VerticalPositionAccuracy',3,'VelocityAccuracy',0.1,...
                'DecayFactor',0.5); 
[GPS_posLLA,GPS_vel,GPS_groundS,GPS_course] = GPS(truePos(1:fs/fsGPS:end,:),trueVel(1:fs/fsGPS:end,:));
GPS_pos = lla2flat(GPS_posLLA,refLoc(1:2),0,-refLoc(3));
tGPS = 0:1/fsGPS:1/fsGPS*(length(GPS_pos(:,1))-1);
%Ideal
% GPSI = gpsSensor('SampleRate',fsGPS,...
%                 'ReferenceFrame','NED','ReferenceLocation',refLoc);
% [GPS_posLLAI,GPS_velI,GPS_groundSI,GPS_courseI] = GPSI(truePos(1:fs/fsGPS:end,:),trueVel(1:fs/fsGPS:end,:));
% GPS_posI = lla2flat(GPS_posLLAI,refLoc(1:2),0,-refLoc(3));
%% Generate syntethic data IMU
fsIMU = 100;
magneticField = [21.9553 -0.7901 33.7241];
% Real
accPar = accelparams('MeasurementRange',(4*9.81),'Resolution',0.0012, ...
                     'ConstantBias',0,... 0.19,...%0.4905,...
                     'AxesMisalignment',0.5,...
                     'NoiseDensity',0.0012356,...%0.0023,...
                     'BiasInstability',0.05,...$ 0.05,...%
                     'RandomWalk',0.001,...%0.1,...
                     'TemperatureBias',0,...%0.0102,...
                     'TemperatureScaleFactor',0);...%0.026);   
gyroPar = gyroparams('MeasurementRange',17.4533,'Resolution',5.3211e-04, ...
                     'ConstantBias',0,...0.0545,...%0.0873,...
                     'AxesMisalignment',0.5,...
                     'NoiseDensity',0.00043633,...2.6180e-04,...
                     'BiasInstability',0.005,...%0.05,...%
                     'RandomWalk',0.001,...%0.1,...
                     'TemperatureBias',0,...%8.7266e-04,
                     'TemperatureScaleFactor',0,...%,0.5236,...
                     'AccelerationBias',0.003);
magPar = magparams('MeasurementRange',4900,'Resolution',0.15, ...
                   'ConstantBias',0,...100,...%0,...
                   'AxesMisalignment',0.5,...%1,...
                   'NoiseDensity',0.0424,...%0.003,...
                   'BiasInstability',0.03,...%0.05,...
                   'RandomWalk',0.03,...%0.1,...
                   'TemperatureBias',0,...%0.002,
                   'TemperatureScaleFactor',0);
IMU = imuSensor('accel-gyro-mag', 'ReferenceFrame', 'NED', 'SampleRate', fsIMU,...
                'Temperature', 25,...
                'MagneticField', magneticField,...
                'Accelerometer',accPar,'Gyroscope',gyroPar,'Magnetometer',magPar);
[IMU_acc,IMU_angV,IMU_magF] = IMU(trueAcc,trueAngV,trueOri);
tIMU = tTrue;
% Ideal
% accParI = accelparams;
% gyroParI = gyroparams;
% magParI = magparams;
% IMUI = imuSensor('accel-gyro-mag', 'ReferenceFrame', 'NED', 'SampleRate', fsIMU,...
%                 'Temperature', 25,...
%                 'MagneticField', magneticField,...
%                 'Accelerometer',accParI,'Gyroscope',gyroParI,'Magnetometer',magParI);
% [IMU_accI,IMU_angVI,IMU_magFI] = IMUI(trueAcc,trueAngV,trueOri);
%% Save sensor data to struct
measurements.nSamplesMax = length(tIMU);
measurements.timeMaxFs = tIMU;
measurements.fsIMU = fsIMU;
measurements.fsAlt = fsALT;
measurements.fsGPS = fsGPS;
measurements.altData = ALT_alt;
measurements.accelData = IMU_acc;
measurements.gyroData = IMU_angV;
measurements.magnetoData = IMU_magF;
measurements.gpsPosData = GPS_posLLA;
measurements.gpsVelData = GPS_vel;
measurements.refPoints = refLoc;
%Save
filename = 'traj2.mat';
save(strcat('.\FlightSessions\',filename),'measurements');
% %Load
% [file,path] = uigetfile('.\FlightSessions\*.mat');
% load(strcat(path,file),'mf');
% newS = measurements;
%% Plot sensor data
%plotSensorData(tALT,ALT_alt,ALT_altI,tGPS,GPS_pos,GPS_posI,tIMU,IMU_acc,IMU_accI,IMU_angV,IMU_angVI,IMU_magF,IMU_magFI);

% figure
% plot(tGPS,GPS_pos(:,3));
% hold on;
% plot(tGPS,GPS_posI(:,3));
% plot(tALT,ALT_alt);
% plot(tALT,ALT_altI);
% title('Altitud GPS y altímetro')
% xlabel('Tiempo [s]')
% ylabel('Altitud [m]')
% legend({'GPS real','GPS ideal','Altímetro real','Altímetro ideal',})

% plot(tGPS,GPS_pos(:,1:2));
% hold on;
% plot(tGPS,GPS_posI(:,1:2));
% title('Posición GPS horizontal')
% xlabel('Tiempo [s]')
% ylabel('Posición [m]')
% legend({'x real','y real','x ideal','y ideal',})
%% Run navigation filter (EKF asynchronous)
N = length(IMU_acc(:,1)); %number of discrete samples on the sensor with higher frequency
fusionfilt = insfilterAsync('ReferenceLocation', refLoc);
% Initialize state vector --> Trampa: vamos a iniciar las condiciones iniciales con los valores reales de la trayectoria
initstate = zeros(28,1);
initstate(1:4) = compact(trueOri(1,:)); %orientation
initstate(5:7) = trueAngV(1,:); %angular velocity
initstate(8:10) = truePos(1,:); %position
initstate(11:13) = trueVel(1,:); %velocity
initstate(14:16) = trueAcc(1,:); %acceleration
initstate(17:19) = IMU.Accelerometer.ConstantBias; %accelerometer bias(m/s2)
initstate(20:22) = IMU.Gyroscope.ConstantBias; %gyroscope bias(rad/s)
initstate(23:25) = IMU_magF(1,:); %geomagnetic vector
initstate(26:28) = IMU.Magnetometer.ConstantBias; %magnetometer bias (uT)
fusionfilt.State = initstate;
fusionfilt.StateCovariance = 1e-5*eye(numel(fusionfilt.State));
% Process noise
fusionfilt.QuaternionNoise;% = 1e-2; 
fusionfilt.AngularVelocityNoise;% = 100;
fusionfilt.PositionNoise;
fusionfilt.VelocityNoise;
fusionfilt.AccelerationNoise;% = 100;
fusionfilt.GyroscopeBiasNoise;% = 1e-7;
fusionfilt.AccelerometerBiasNoise;% = 1e-7;
fusionfilt.GeomagneticVectorNoise;
fusionfilt.MagnetometerBiasNoise;% = 1e-7;
% Sensor noise
Rmag = 50;
Rvel = GPS.VelocityAccuracy^2;
Racc = 100;
Rgyro = 0.01;
Rpos = [GPS.HorizontalPositionAccuracy^2 GPS.HorizontalPositionAccuracy^2 GPS.VerticalPositionAccuracy^2];
% Run the nav filter loop
estPos = zeros(N,3);
estOri = zeros(N,1,'quaternion');
dt = 1/fsIMU;
gpsIdx = 1;
for ii = 1:N 
    % Predict the filter forward one time step
    predict(fusionfilt,dt);
    % Fuse accelerometer, gyroscope and magnetometer readings
    fuseaccel(fusionfilt,IMU_acc(ii,:),Racc);
    fusegyro(fusionfilt,IMU_angV(ii,:),Rgyro);
    fusemag(fusionfilt,IMU_magF(ii,:),Rmag); 
    % Fuse GPS each 20 IMU samples
    if (mod(ii,fix(fsIMU/fsGPS)) == 0)
        fusegps(fusionfilt,GPS_posLLA(gpsIdx,:),Rpos,GPS_vel(gpsIdx,:),Rvel);
        gpsIdx = gpsIdx + 1;        
    end    
    % Log the current pose estimate
    [estPos(ii,:),estOri(ii)] = pose(fusionfilt);
end

bodyFrameAxisE = zeros(3,3,nElements);
for c=1:nElements
    bodyFrameAxisE(:,:,c) = rotatepoint(estOri(c),NED_axis);
end
%% Plot ideal and estimated trajectories
hold(ax,'on');
%plot3(ax,Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'LineStyle','none','Marker','.','MarkerEdgeColor','k');
plot3(ax,truePos(:,1),truePos(:,2),truePos(:,3),'k-.');
plot3(ax,estPos(:,1),estPos(:,2),estPos(:,3),'Color','r');
hold(ax,'off');
%% Play animation
% FPS = 25; %animation FPS (max 30 without too much errors)
% hold(ax,'on');
% % Coordinate system true
% qBFTx = quiver3(ax,0,0,0,0,0,0,'Color',[0.6350 0.0780 0.1840],'LineWidth',1.5,'MaxHeadSize',1);
% qBFTy = quiver3(ax,0,0,0,0,0,0,'Color',[0.4660 0.6740 0.1880],'LineWidth',1.5,'MaxHeadSize',1);
% qBFTz = quiver3(ax,0,0,0,0,0,0,'Color',[0 0.4470 0.7410],'LineWidth',1.5,'MaxHeadSize',1);
% gcTrue = plot3(ax,NaN,NaN,NaN,'Marker','o','MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','k');
% %Coordinate system estimate
% qBFEx = quiver3(ax,0,0,0,0,0,0,'Color','r','LineWidth',1.5,'MaxHeadSize',1);
% qBFEy = quiver3(ax,0,0,0,0,0,0,'Color','g','LineWidth',1.5,'MaxHeadSize',1);
% qBFEz = quiver3(ax,0,0,0,0,0,0,'Color','b','LineWidth',1.5,'MaxHeadSize',1);
% gcEst = plot3(ax,NaN,NaN,NaN,'Marker','o','MarkerSize',4,'MarkerEdgeColor','r','MarkerFaceColor','r');
% %
% ratePlotDiv = round(fs/FPS); % reduces the FPS to plot
% r = rateControl(fs/ratePlotDiv); % fs/ratePlotDiv = FPS (but a round is needes if division is not exact)
% pause(3);
% for c=0:ratePlotDiv:nElements
%     if (c == 0)
%         c=1;
%     end
%    qBFTx.XData = truePos(c,1);
%    qBFTx.YData = truePos(c,2); 
%    qBFTx.ZData = truePos(c,3); 
%    qBFTx.UData = bodyFrameAxisT(1,1,c);
%    qBFTx.VData = bodyFrameAxisT(1,2,c);
%    qBFTx.WData = bodyFrameAxisT(1,3,c); 
%    qBFTy.XData = truePos(c,1);
%    qBFTy.YData = truePos(c,2); 
%    qBFTy.ZData = truePos(c,3);
%    qBFTy.UData = bodyFrameAxisT(2,1,c);
%    qBFTy.VData = bodyFrameAxisT(2,2,c);
%    qBFTy.WData = bodyFrameAxisT(2,3,c);
%    qBFTz.XData = truePos(c,1);
%    qBFTz.YData = truePos(c,2); 
%    qBFTz.ZData = truePos(c,3);
%    qBFTz.UData = bodyFrameAxisT(3,1,c);
%    qBFTz.VData = bodyFrameAxisT(3,2,c);
%    qBFTz.WData = bodyFrameAxisT(3,3,c);
%    gcTrue.XData = truePos(c,1);
%    gcTrue.YData = truePos(c,2); 
%    gcTrue.ZData = truePos(c,3);
%    
%    qBFEx.XData = estPos(c,1);
%    qBFEx.YData = estPos(c,2); 
%    qBFEx.ZData = estPos(c,3); 
%    qBFEx.UData = bodyFrameAxisE(1,1,c);
%    qBFEx.VData = bodyFrameAxisE(1,2,c);
%    qBFEx.WData = bodyFrameAxisE(1,3,c); 
%    qBFEy.XData = estPos(c,1);
%    qBFEy.YData = estPos(c,2); 
%    qBFEy.ZData = estPos(c,3);
%    qBFEy.UData = bodyFrameAxisE(2,1,c);
%    qBFEy.VData = bodyFrameAxisE(2,2,c);
%    qBFEy.WData = bodyFrameAxisE(2,3,c);
%    qBFEz.XData = estPos(c,1);
%    qBFEz.YData = estPos(c,2); 
%    qBFEz.ZData = estPos(c,3);
%    qBFEz.UData = bodyFrameAxisE(3,1,c);
%    qBFEz.VData = bodyFrameAxisE(3,2,c);
%    qBFEz.WData = bodyFrameAxisE(3,3,c);
%    gcEst.XData = estPos(c,1);
%    gcEst.YData = estPos(c,2); 
%    gcEst.ZData = estPos(c,3);
%    
%    waitfor(r);
% end
% stats = statistics(r)
% hold(ax,'off');

%%
posErr = truePos - estPos;
qErr = rad2deg(dist(trueOri,estOri));
pRMS = sqrt(mean(posErr.^2))
qRMS = sqrt(mean(qErr.^2))

figure;
% subplot(2,1,1);
plot(tTrue,posErr);
title('Error de posición a 50km/h')
xlabel('Tiempo [s]')
ylabel('Error posición [m]')
legend({'x','y','z'})
% subplot(2,1,2);
% plot(tTrue,qErr);
% title('Error de orientación')
% xlabel('Tiempo [s]')
% ylabel('Error orientación [deg]')
















