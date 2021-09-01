function [estPos,estOri,estVel] = runKalmanEstimation(sensorData)
    % sensorData.nSamplesMax
    % sensorData.timeMaxFs
    % sensorData.fsIMU
    % sensorData.fsAlt
    % sensorData.fsGPS
    % sensorData.altData
    % sensorData.accelData
    % sensorData.gyroData
    % sensorData.magnetoData
    % sensorData.gpsPosData
    % sensorData.gpsVelData
    % sensorData.refPoints
    
    kFilter = insfilterAsync('ReferenceLocation', sensorData.refPoints(1,:));
    %Initialize state vector
    initstate = zeros(28,1);
    initstate(1:4) = [1 0 0 0]; %orientation
    initstate(5:7) = [0 0 0]; %angular velocity
    initstate(8:10) = lla2flat(sensorData.gpsPosData(1,:),sensorData.refPoints(1,1:2),0,-sensorData.refPoints(1,3));%position
    initstate(11:13) = sensorData.gpsVelData(1,:); %velocity
    initstate(14:16) = [0 0 0]; %acceleration
    initstate(17:19) = 0; %accelerometer bias(m/s2)
    initstate(20:22) = 0; %gyroscope bias(rad/s)
    initstate(23:25) = sensorData.magnetoData(1,:); %geomagnetic vector
    initstate(26:28) = 0; %magnetometer bias (uT)
    kFilter.State = initstate;
    %Initialize covariance matrix
    kFilter.StateCovariance = 1e-5*eye(numel(kFilter.State));
    %Initialize process noise
    kFilter.QuaternionNoise;% = 1e-2; 
    kFilter.AngularVelocityNoise;% = 100;
    kFilter.PositionNoise;
    kFilter.VelocityNoise;
    kFilter.AccelerationNoise;% = 100;
    kFilter.GyroscopeBiasNoise;% = 1e-7;
    kFilter.AccelerometerBiasNoise;% = 1e-7;
    kFilter.GeomagneticVectorNoise;
    kFilter.MagnetometerBiasNoise;% = 1e-7;
    %Sensor noise
    Rmag = 50;
    Rvel = [0.1^2 0.1^2 0.1^2];
    Racc = 100;
    Rgyro = 0.01;
    Rpos = [1.6^2 1.6^2 3^2];
    
    %Run the filter
    estPos = zeros(sensorData.nSamplesMax,3);
    estOri = zeros(sensorData.nSamplesMax,1,'quaternion');
    estVel = zeros(sensorData.nSamplesMax,3);
    dt = 1/sensorData.fsIMU;
    gpsIdx = 1;
    for ii = 1:sensorData.nSamplesMax 
        % Predict the filter forward one time step
        predict(kFilter,dt);
        % Fuse accelerometer, gyroscope and magnetometer readings
        fuseaccel(kFilter,sensorData.accelData(ii,:),Racc);
        fusegyro(kFilter,sensorData.gyroData(ii,:),Rgyro);
        fusemag(kFilter,sensorData.magnetoData(ii,:),Rmag); 
        % Fuse GPS each 20 IMU samples
        if (mod(ii,fix(sensorData.fsIMU/sensorData.fsGPS)) == 0)
            fusegps(kFilter,sensorData.gpsPosData(gpsIdx,:),Rpos,sensorData.gpsVelData(gpsIdx,:),Rvel);
            gpsIdx = gpsIdx + 1;        
        end    
        % Log the current pose estimate
        [estPos(ii,:),estOri(ii),estVel(ii,:)] = pose(kFilter);
    end
end

