%INPUTS
    %Track Width, Wheel OD, Wheel thickness, Aeroshell Contour, CG Height, CG XY, Wheelbase, RoC
%OUTPUTS
    %Wheel steer angles, static roll stability, maximum capable lateral g's 

%%DATA
R = 6000; %turn radius mm
TFront = 900 ; %mm
TRear =  TFront; %mm
L = 1675; %wheel base mm
TireW = 100; %mm
TireOD = 566; %mm
tireRotOffset = 52; % KPI lateral offeset from tire edge mm
CGX = 0; %mm
CGY = 3; %mm
CGZ = 380; %mm
bankAngle = 45; %degrees
r = 8.5; %figure-8 radius m
t = 8; %figure-8 time s

%%PROGRAM
%Calculate ackerman steering angles in degrees
insideAngle = atand(L/(R-TFront/2));
outsideAngle = atand(L/(R+TFront/2));

%Calculate laterial acc required during figure 8
aFig8 = (4*pi^2*r/t^2)/9.81;

%Calculate max capable braking/accleration force
LongLT=0.5;
Ax=LongLT*L/CGZ;
%Calculate max capable lateral acceleration
LLT=0.5;
Ay=LLT*((TFront+TRear)/2)/CGZ;

%Display major parameters
disp(['Maximum cornering capacity: ', num2str(Ay), 'g'])
disp(['Maximum braking capacity: ', num2str(Ax), 'g'])
disp(' ')
disp(['CG Height is set to ', num2str(CGZ),' mm'])
disp(['Wheel base is set to ', num2str(L),' mm'])
disp(['Front track width is set to ', num2str(TFront),' mm'])
disp(['Turning radius is set to ', num2str(R),' mm'])
disp(' ')
disp(['Inside wheel angle is: ', num2str(insideAngle),' degrees'])
disp(['Outside wheel angle is: ', num2str(outsideAngle), ' degrees'])
disp(' ')
%%Wheel Geometry
%x coordinates for front wheel in the second quadrant
xf1=-(TFront/2-TireW/2); 
xf2=-(TFront/2+TireW/2);
xf3=xf2;
xf4=xf1;

%x coordinates for rear wheel in the second quadrant
xr1= -(TRear/2-TireW/2);
xr2= -(TRear/2+TireW/2);
xr3=xr2;
xr4=xr1;

wheelXfq2 = [xf1,xf2,xf3,xf4]; %front left (quadrant 2) wheel x coordinates
wheelXrq2 = [xr1,xr2,xr3,xr4]; %rear left (quadrant 2) wheel x coordinates
wheelYq2 = [(L/2-TireOD/2),(L/2-TireOD/2),(L/2+TireOD/2),(L/2+TireOD/2)]; %front axle (quadrant 1&2) wheel y coordinates

%%Wheel steering top view graphs
hold on
figure(1)
title('TOP VIEW: Wheels in Full Steering')
%graph wheels in their static position
insideFWheel = polyshape(wheelXfq2,wheelYq2); 
outsideFWheel = polyshape(-wheelXfq2,wheelYq2);
insideRWheel = polyshape(wheelXrq2,-wheelYq2);
outsideRWheel = polyshape(-wheelXrq2,-wheelYq2);
%Rotate front wheels about the upright center axis (ASSUME ZERO KPI  & CASTER)
insideFWheelRot= rotate(insideFWheel, insideAngle, [wheelXfq2(1)+tireRotOffset,(wheelYq2(3)+wheelYq2(2))/2]);
outsideFWheelRot= rotate(outsideFWheel, outsideAngle, [-wheelXfq2(1)-tireRotOffset,(wheelYq2(3)+wheelYq2(2))/2]);
%plot the center of rotation of the wheels
plot(wheelXfq2(1)+tireRotOffset,(wheelYq2(3)+wheelYq2(2))/2,'r.', 'MarkerSize', 20);
plot(-wheelXfq2(1)-tireRotOffset,(wheelYq2(3)+wheelYq2(2))/2,'r.', 'MarkerSize', 20);
%plot all configurations
plot([insideFWheel,outsideFWheel,insideRWheel,outsideRWheel,insideFWheelRot,outsideFWheelRot]);
%plot track width and wheel base lines
plot([(xf2+xf1)/2 (-xf2+(-xf1))/2], [(wheelYq2(3)+wheelYq2(2))/2 (wheelYq2(3)+wheelYq2(2))/2], 'g', 'LineWidth', 2);
plot([(xr2+xr1)/2 (-xr2+(-xr1))/2], [-(wheelYq2(3)+wheelYq2(2))/2 -(wheelYq2(3)+wheelYq2(2))/2], 'g', 'LineWidth', 2);
plot([0 0], [-(wheelYq2(3)+wheelYq2(2))/2 (wheelYq2(3)+wheelYq2(2))/2], 'y', 'LineWidth', 2);
axis equal
hold off

%%Front banked view
figure(2)
title('FRONT VIEW: Average Single Axle Banked at 45deg')
%Average the front and rear axles track width
avgWheelXfq2 = (wheelXfq2+wheelXrq2)/2;
%plot wheels at zero banking
leftFWheel = polyshape(avgWheelXfq2,[0 0 TireOD TireOD]);
rightFWheel = polyshape(-avgWheelXfq2,[0 0 TireOD TireOD]);
%plot wheels at banking angle
insideFWheel2Rot = rotate(leftFWheel, bankAngle, [avgWheelXfq2(2) 0]);
outsideFWheel2Rot = rotate(rightFWheel, bankAngle, [avgWheelXfq2(2) 0]);
%Run a rotation transformation on road ground line and CG coords based on bank angle
CGXRot = (CGX-avgWheelXfq2(2))*cosd(bankAngle) - (CGZ-0)*sind(bankAngle)+avgWheelXfq2(2);
CGZRot = (CGX-avgWheelXfq2(2))*sind(bankAngle) + (CGZ-0)*cosd(bankAngle)+0;
groundRotX = (-avgWheelXfq2(2)-avgWheelXfq2(2))*cosd(bankAngle)+avgWheelXfq2(2);
groundRotY = (-avgWheelXfq2(2)-avgWheelXfq2(2))*sind(bankAngle);

hold on
%plot([avgWheelXfq2(2) -avgWheelXfq2(2)],[0 0],'black', 'LineWidth',3); %Plot horizontal road ground line
plot([avgWheelXfq2(2) groundRotX],[0 groundRotY],'black', 'LineWidth',3); %Plot banked road ground line
%plot(0,CGZ,'g*', 'MarkerSize', 10); %plot horizontal CG location
plot(CGXRot,CGZRot,'r*', 'MarkerSize', 10); %plot rotated CG location
plot([CGXRot CGXRot],[CGZRot 0],'b', 'LineWidth',1); %plot vertical line from rotated CG location
%plot([leftFWheel,rightFWheel]) %plot horizontal wheels
plot([insideFWheel2Rot,outsideFWheel2Rot]);%plot banked angle wheels

%Calculate Static Stability Safety Factor
offset_contactPatch = (CGXRot-avgWheelXfq2(2))/cosd(bankAngle)-TireW/2;
offset_FS = offset_contactPatch/TFront+1;
disp(['Roll Stability Factor of Safty is ', num2str(offset_FS)])

axis equal
hold off
