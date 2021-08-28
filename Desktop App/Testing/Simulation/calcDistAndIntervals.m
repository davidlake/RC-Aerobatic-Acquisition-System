function [totalDist,timeIntervals] = calcDistAndIntervals(waypoints,meanSpeed)
% waypoints: Nx3 matrix [m]
% meanSpeed: single value [km/h]
% totalDist: [m]
% timeIntervals: [s]
    nElements = length(waypoints(:,1));
    distM = squareform(pdist(waypoints));
    distIntervals = zeros(nElements-1,1);
    for i=1:(nElements-1)
        distIntervals(i) = distM(i,i+1);
    end
    totalDist = sum(distIntervals);
    meanSpeedN = meanSpeed*1000/3600;
    timeIntervals = distIntervals/meanSpeedN;
end

