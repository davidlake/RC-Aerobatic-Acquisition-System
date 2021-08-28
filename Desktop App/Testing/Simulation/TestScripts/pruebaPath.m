Waypoints = [0,0,0;...
            1,0,1;...
            0,0,2];
% Orientation = [0,0,0;...
%             0,-90,0;...
%             0,-180,0];
% quatOrientation = quaternion(Orientation,'eulerd','ZYX','frame');
% Velocities = [1,0,0;...
%              0,0,1;...
%             -1,0,0];
TimeOfArrival = [0;1;2];
trajectory = waypointTrajectory(Waypoints,TimeOfArrival,'SampleRate',10);
truePosition = zeros(trajectory.SampleRate*trajectory.TimeOfArrival(end)-1,3); 
c = 1;
while ~isDone(trajectory)
   truePosition(c,:) = trajectory();
   c = c + 1;
end
plot3(Waypoints(:,1),Waypoints(:,2),Waypoints(:,3),'LineStyle','none','Marker','o','MarkerEdgeColor','r');
hold on;
plot3(truePosition(:,1),truePosition(:,2),truePosition(:,3),'Color','k');
daspect([1 1 1]);
grid on;
xlabel('x');
ylabel('y');
zlabel('z');