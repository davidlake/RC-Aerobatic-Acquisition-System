function [positionVect,theoEndPoint,theoDirOut,residTOut] = circularTrajectory(fs,linearVelocity,radius,angleTravel,theoInitPoint,directionIn,rotation,residTIn)
    % fs: sampling freq in Hz
    % linearVelocity: in m/s
    % radius: in m
    % angFraction: in degrees (360 full circle)
    % theoInitPoint: 1x3 point xyz (theoretical starting point)
    % direction: [xrot yrot zrot] in deg direction vector defined by frame angles
    % residTIn: time residual from previous trajectory discretization
    
    angleTravel = deg2rad(angleTravel);
    angVelocity = linearVelocity/radius;
    totalTime = angleTravel/angVelocity-residTIn;
    nElements = floor(totalTime*fs)+1;
    deltaAng = angVelocity/fs;
    residTOut = 1/fs-(totalTime - (nElements-1)/fs);
    initAng = residTIn*angVelocity;
    
    directionInr = directionIn;
    directionInr(1) = directionInr(1)+rotation;
    
    angleVect = (initAng:deltaAng:angleTravel)-(pi/2);
    positionVect = [transpose(radius*cos(angleVect)),transpose(radius*sin(angleVect)+radius),zeros(nElements,1)];
    positionVect = rotatepoint(quaternion(directionInr,'eulerd','XYZ','point'),positionVect);
    positionVect = positionVect + theoInitPoint;

    anglePoint = angleTravel-(pi/2);
    theoEndPoint = [radius*cos(anglePoint),radius*sin(anglePoint)+radius,0];
    theoEndPoint = rotatepoint(quaternion(directionInr,'eulerd','XYZ','point'),theoEndPoint);
    theoEndPoint = theoEndPoint + theoInitPoint;
    
    q1 = quaternion([0,-rad2deg(angleTravel),0],'eulerd','XYZ','point');
    q2 = quaternion([directionIn(1)-(90-rotation),directionIn(2),directionIn(3)],'eulerd','XYZ','point');
    
    theoDirOut = eulerd(q2*q1,'XYZ','point');
    
end

