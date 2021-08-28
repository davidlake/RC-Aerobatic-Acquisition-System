fs = 10;
linearVelocity = 5; %m/s
length = 100; %m
initPoint = [-250,0,0];
directionIn = [0,0,0];
residTIn = 0;
[position1,theoEndPoint,theoDirOut,residTOut] = lineTrajectory(fs,linearVelocity,length,initPoint,directionIn,residTIn);
radius = 50;
angleTravel = 90;
rotation = 90;
[position2,theoEndPoint,theoDirOut,residTOut] = circularTrajectory(fs,linearVelocity,radius,angleTravel,theoEndPoint,theoDirOut,rotation,residTOut);
radius = 50;
angleTravel = 90;
rotation = -90;
[position3,theoEndPoint,theoDirOut,residTOut] = circularTrajectory(fs,linearVelocity,radius,angleTravel,theoEndPoint,theoDirOut,rotation,residTOut);
length = 100; %m
[position4,theoEndPoint,theoDirOut,residTOut] = lineTrajectory(fs,linearVelocity,length,theoEndPoint,theoDirOut,residTOut);
radius = 100;
angleTravel = 180;
rotation = -90;
[position5,theoEndPoint,theoDirOut,residTOut] = circularTrajectory(fs,linearVelocity,radius,angleTravel,theoEndPoint,theoDirOut,rotation,residTOut);
length = 100; %m
[position6,theoEndPoint,theoDirOut,residTOut] = lineTrajectory(fs,linearVelocity,length,theoEndPoint,theoDirOut,residTOut);

p = [position1;position2;position3;position4;position5;position6];

plot3(p(:,1),p(:,2),p(:,3));
axis equal
xlabel('x');
ylabel('y');
zlabel('z');
grid on;