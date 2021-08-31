function [positionVect,theoEndPoint,theoDirOut,residTOut] = lineTrajectory(fs,velocity,length,theoInitPoint,directionIn,residTIn)
    % fs: sampling freq in Hz
    % speed: in m/s
    % length: in m
    % theoInitPoint: 1x3 point xyz (theoretical starting point)
    % direction: [xrot yrot zrot] in deg direction vector defined by frame angles
    % residTIn: time residual from previous trajectory discretization
    
    totalTime = length/velocity-residTIn;
    nElements = floor(totalTime*fs)+1;
    deltaL = velocity/fs;
    residTOut = 1/fs-(totalTime - (nElements-1)/fs);
    initL = residTIn*velocity;
    
    positionVect = [transpose(initL:deltaL:length),zeros(nElements,1),zeros(nElements,1)];
    positionVect = rotatepoint(quaternion(directionIn,'eulerd','XYZ','point'),positionVect);
    positionVect = positionVect + theoInitPoint;
    
    theoEndPoint = [length,0,0];
    theoEndPoint = rotatepoint(quaternion(directionIn,'eulerd','XYZ','point'),theoEndPoint);
    theoEndPoint = theoEndPoint + theoInitPoint;
    
    theoDirOut = directionIn;
end

