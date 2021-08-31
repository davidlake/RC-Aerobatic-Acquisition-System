function plotSensorData(tALT,ALT_alt,ALT_altI,tGPS,GPS_pos,GPS_posI,tIMU,IMU_acc,IMU_accI,IMU_angV,IMU_angVI,IMU_magF,IMU_magFI)
    figure;
%% Position xy GPS
    subplot(3,5,[1,2]);
    plot(tGPS,GPS_pos(:,1:2));
    hold on;
    plot(tGPS,GPS_posI(:,1:2));
    title('Posición GPS horizontal')
    xlabel('Tiempo [s]')
    ylabel('Posición [m]')
    legend({'x real','y real','x ideal','y ideal',})
%% Altitude GPS and ALTIMETER
    subplot(3,5,[6,12]);
    plot(tGPS,GPS_pos(:,3));
    hold on;
    plot(tGPS,GPS_posI(:,3));
    plot(tALT,ALT_alt);
    plot(tALT,ALT_altI);
    title('Altitud GPS y altímetro')
    xlabel('Tiempo [s]')
    ylabel('Altitud [m]')
    legend({'GPS real','GPS ideal','Altímetro real','Altímetro ideal',})
%% Acceleration IMU
    subplot(3,5,3);
    plot(tIMU,IMU_acc(:,1));
    hold on;
    plot(tIMU,IMU_accI(:,1));
    title('Aceleración - x')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [m/s2]')
    legend({'Real','Ideal'})
    subplot(3,5,4);
    plot(tIMU,IMU_acc(:,2));
    hold on;
    plot(tIMU,IMU_accI(:,2));
    title('Aceleración - y')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [m/s2]')
    legend({'Real','Ideal'})
    subplot(3,5,5);
    plot(tIMU,IMU_acc(:,3));
    hold on;
    plot(tIMU,IMU_accI(:,3));
    title('Aceleración - z')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [m/s2]')
    legend({'Real','Ideal'})
%% Angular velocity GYRO
    subplot(3,5,8);
    plot(tIMU,rad2deg(IMU_angV(:,1)));
    hold on;
    plot(tIMU,rad2deg(IMU_angVI(:,1)));
    title('V. angular - x')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [deg/s]')
    legend({'Real','Ideal'})    
    subplot(3,5,9);
    plot(tIMU,rad2deg(IMU_angV(:,2)));
    hold on;
    plot(tIMU,rad2deg(IMU_angVI(:,2)));
    title('V. angular - y')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [deg/s]')
    legend({'Real','Ideal'})     
    subplot(3,5,10);
    plot(tIMU,rad2deg(IMU_angV(:,3)));
    hold on;
    plot(tIMU,rad2deg(IMU_angVI(:,3)));
    title('V. angular - z')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [deg/s]')
    legend({'Real','Ideal'})     
%%  Magnetic field MAGNETOMETER
    subplot(3,5,13);
    plot(tIMU,IMU_magF(:,1));
    hold on;
    plot(tIMU,IMU_magFI(:,1));
    title('Flujo mag. - x')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [uT]')
    legend({'Real','Ideal'}) 
    subplot(3,5,14);
    plot(tIMU,IMU_magF(:,2));
    hold on;
    plot(tIMU,IMU_magFI(:,2));
    title('Flujo mag. - y')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [uT]')
    legend({'Real','Ideal'})     
    subplot(3,5,15);
    plot(tIMU,IMU_magF(:,3));
    hold on;
    plot(tIMU,IMU_magFI(:,3));
    title('Flujo mag. - z')
    xlabel('Tiempo [s]')
    ylabel('Aceleración [uT]')
    legend({'Real','Ideal'})     
end

