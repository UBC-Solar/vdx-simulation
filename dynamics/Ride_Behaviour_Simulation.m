%Ride Behaviour Simulation 
%By: Sina Allen
%Draft Release Date: 2023-02-20
%OUTPUTS:
%   -Optimal Spring Rate based on 1:1 motion ratio
%   -wheel loads at various suspension modes
%Assumptions:
%   -treat car as single mass body (sprung + unsprung)
%   -all calculations are done at static position (ride height)
%   -steady state turn
%   -tire deflection included in calcs (assuming it's accurate)
%   -CG and RC are at the center line of the vehicle
%   -Bias towards understeer is made however not directly (made through roll rate distribution and RC distribution)

%%NOTES
%Things to be distributed
%   -weight --> 52.5% rear, 47.5% front to start --> **NEEDS OPTIMIZING**
%   -RC heights --> based on geometry constraints. Ensure rear is lower
%    than front.
%   -roll rates --> 55% front, 45% rear. 
%   -Total Lateral Load Transfer Distribution --> Something we find out.
%    Ensure front is higher than rear.
%**GOAL IS TO BIAS TOWARD UNDERSTEER**

%%TODO
%   -vectorize all front/rear variables and calculatons to reduce size

%%Setup%%
%Load Data Sheet (FULL LINK:https://docs.google.com/spreadsheets/d/1UxrHRz9iydKkYqhXAQWLDXY1FJoSzHCgv_TjBJgwtKQ/edit#gid=2032725398)
ID = '1UxrHRz9iydKkYqhXAQWLDXY1FJoSzHCgv_TjBJgwtKQ';
sheet_name = 'Ride Behaviour ';
url_name = sprintf('https://docs.google.com/spreadsheets/d/%s/gviz/tq?tqx=out:csv&sheet=%s',...
    ID, sheet_name);
data = webread(url_name);

%unit conversion factors
m_ft = 3.28;
ft_m = 0.3408;
kg_lb = 2.20462;
lb_kg = 0.453592;

%%DATA%%
%Distributions
w_dstr_front = data.InputBasicParamsValue(6)/100; %[% decimal]
k_roll_dstr_front = data.InputFront(1)/100; %[% decimal]

%Rates
k_tire = data.InputFront(2); %[lbs/in]
f_ride_goal_front = data.InputFront(3); %[HZ]
f_ride_goal_rear = data.InputRear(3); 

%Weights
w_total = data.InputBasicParamsValue(5); %[lbs] 
w_front = w_total * w_dstr_front; %front axle weight [lbs]
w_rear = w_total * (1-w_dstr_front); %rear axle weight [lbs]

%Geometry
rideHeight = data.InputBasicParamsValue(2); %[ft]
trackWidth = data.InputBasicParamsValue(3); %[ft]
wheelBase = data.InputBasicParamsValue(4); %[ft]
aeroWidth = 4.05; %[ft]
aeroFrontOverhang = 1.6*m_ft; %[ft]
CG_height = data.InputBasicParamsValue(1); %[ft]
l = data.InputBasicParamsValue(4); %[ft] 
b = w_front*l/w_total; %[ft] 
a = l-b; %[ft] 
RC_height_front = data.InputFront(4); %[ft]
RC_height_rear = data.InputRear(4); %[ft] 
RC_height_atCG = RC_height_rear + b/l*(RC_height_front-RC_height_rear);
cornering_displacement_wheel_front = data.InputFront(5); %[in]
cornering_displacement_wheel_rear = data.InputRear(5); %[in]
CG_NRA = CG_height - (b/l*(RC_height_front-RC_height_rear)); %input[ft] output[ft]
motion_ratio = data.InputFront(7);

%Kinematics
a_y = data.InputFront(6); %[g] max cornering acceleration based roll over requirement (see track width sim for details)
max_roll_angle = atan((rideHeight-RC_height_atCG)/(aeroWidth/2)) + atan(RC_height_atCG/(aeroWidth/2)); %[rad]
roll_gradient = max_roll_angle/(a_y*1.5); %[rad/g] exprience 2/3 of max roll through max lateral acceleration

a_x = data.InputFront(8); %[g] max braking acceleration from suspension loading regs

%%CALCULATIONS 

%calculate LoLT
LoLT = w_total*CG_height*a_x/wheelBase; %[lbs] assuming laterally centered CG

%estimate roll rates based on desired roll gradient (initial constraint) and front/rear roll
%moment distribution
k_roll_total = w_total*CG_NRA/roll_gradient; %inputs[lbs, ft, rad/g] output[lb.ft/rad] **removed (-)
k_roll_front = k_roll_total * k_roll_dstr_front; %inputs[lb.ft/rad, %] output[lb.ft/rad]
k_roll_rear = k_roll_total - k_roll_front; %inputs[lb.ft/rad] output[lb.ft/rad]
disp(['initial total roll rate: ', num2str(k_roll_total), ' lb.ft/rad'])
disp(['initial front roll rate: ', num2str(k_roll_front), ' lb.ft/rad'])
disp(['initial rear roll rate: ', num2str(k_roll_rear), ' lb.ft/rad'])

%calculate lateral load transfers based on estimated roll rates
LLT_front = a_y*w_total/trackWidth*((CG_NRA*k_roll_front)/(k_roll_front+k_roll_rear)+b/l*RC_height_front); %inputs[g, lbs, ft, lf.ft/rad] output[lbs]
LLT_rear = a_y*w_total/trackWidth*((CG_NRA*k_roll_rear)/(k_roll_front+k_roll_rear)+a/l*RC_height_rear);

%calculate ride rate based on wheel displacement (initial constraint)
k_ride_front = LLT_front/cornering_displacement_wheel_front; %inputs[lbs, inch] output[lbs/in]
k_ride_rear = LLT_rear/cornering_displacement_wheel_rear;

%calculate ride frequency based on ride rate and weight on each wheel. 
f_ride_front = sqrt((k_ride_front*12*32.2)/(w_front/2))/(2*pi); %inputs[lbs/in, lbs] output[Hz]
f_ride_rear = sqrt((k_ride_rear*12*32.2)/(w_rear/2))/(2*pi);
disp(['initial front ride frequnecy: ', num2str(f_ride_front), ' Hz'])
disp(['initial rear ride frequnecy: ', num2str(f_ride_rear), ' Hz'])

if f_ride_rear > f_ride_front
    disp('scaling front ride rate...')
    %calculate new front ride rate so front ride frequency is 25% higher
    %than rear
    k_ride_front = k_ride_front * (f_ride_rear/f_ride_front*1.25)^2; %inputs[lbs/in, lbs] output[Hz]
    %recalculate all ride frequencies
    f_ride_front = sqrt((k_ride_front*12*32.2)/(w_front/2))/(2*pi); %inputs[lbs/in, lbs] output[Hz]
    f_ride_rear = sqrt((k_ride_rear*12*32.2)/(w_rear/2))/(2*pi);
    f_ride_dist = f_ride_front/f_ride_rear;
    disp(['front ride frequency is higher than rear by a factor of: ', num2str(f_ride_dist)])
end

%recalculate roll rates based on ride rates
k_roll_front = 12*k_ride_front*trackWidth^2/2; %inputs[lbs/in, ft] output[lb.ft/rad]
k_roll_rear = 12*k_ride_rear*trackWidth^2/2; 
k_roll_total = k_roll_front + k_roll_rear;
k_roll_dstr_front = k_roll_front/k_roll_total;
disp(['final front roll rate distribution is: ', num2str(k_roll_dstr_front)])

%recalc LLT based on new roll rate
LLT_front = a_y*w_total/trackWidth*((CG_NRA*k_roll_front)/(k_roll_front+k_roll_rear)+b/l*RC_height_front);%inputs[g, lbs, ft, lf.ft/rad] output[lbs]
LLT_rear = a_y*w_total/trackWidth*((CG_NRA*k_roll_rear)/(k_roll_front+k_roll_rear)+a/l*RC_height_rear);

%calculate TLLTD
TLLTD_front = LLT_front / (LLT_rear + LLT_front);
TLLT = w_total*a_y*CG_height/trackWidth;
disp(['Total Lateral Load Distribution at Front: ', num2str(TLLTD_front)])

%calculate new lateral wheel displacements
cornering_displacement_wheel_front = LLT_front/k_ride_front; %inputs[lbs, lbs/in] output[in]
cornering_displacement_wheel_rear = LLT_rear/k_ride_rear;

%calculate longitudinal wheel displacements
braking_displacement_wheel_front = LoLT/k_ride_front; %inputs[lbs, lbs/in] output[in]
braking_displacement_wheel_rear = LoLT/k_ride_rear;

%calculate spring rate from wheel rate 
k_wheel_front = (k_ride_front*k_tire)/(k_tire-k_ride_front); %inputs[lbs/in] output[ilbs/in]
k_wheel_rear = (k_ride_front*k_tire)/(k_tire-k_ride_front);
k_spring_front = k_wheel_front/motion_ratio^2; %inputs[lbs/in] output[lbs/in]
k_spring_rear = k_wheel_rear/motion_ratio^2;
disp(['calculated front spring rate: ', num2str(k_spring_front), ' lbs/in'])
disp(['calculated rear spring rate: ', num2str(k_spring_rear), ' lbs/in'])

%%VERIFICATION
%compare f_ride to f_ride_goal; first check if front is larger than rear;
%if yes then continue; if no scale K_ride_front appropriatly until f_ride_front is larger than f_ride_rear 
if f_ride_rear > f_ride_front
     disp("WARNING: front ride frequency is less than rear.")
end

%check if ride frequency is more or less than 50% of the goal
if (f_ride_front > (f_ride_goal_front*1.5) || f_ride_front < (f_ride_goal_front/1.5)) || (f_ride_rear > (f_ride_goal_front*1.5) || f_ride_rear < (f_ride_goal_front/1.5))
    disp("WARNING: ride frequencies is out of range.")
end

%verify body roll based on new roll rate value
body_roll = w_total*CG_NRA/k_roll_total*a_y;%inputs[lbs, ft, g, lbs.ft/rad] output[rad] **Removed '-'
if abs(body_roll) > abs(max_roll_angle)
    disp("WARNING: body roll is more than the maximum.")
end

%verify body pitch
body_pitch_angle = atan((braking_displacement_wheel_front + braking_displacement_wheel_rear)/(12*wheelBase)); %inputs[in, ft] output[rad]
body_pitch = aeroFrontOverhang*sin(body_pitch_angle); %input [ft, rad] output [ft]
min_wheelbase = (braking_displacement_wheel_front + braking_displacement_wheel_rear)/(12*tan(asin(rideHeight/aeroFrontOverhang)));
if  body_pitch > rideHeight
    disp("WARNING: body pitch is more than maximum - car eats shit")
end

%wheel displacement check to see if they are within range (1-2inch)
if (cornering_displacement_wheel_front > 2.05 || cornering_displacement_wheel_front < 0.95)|| (cornering_displacement_wheel_rear > 2.05 || cornering_displacement_wheel_rear < 0.95)
    disp("WARNING: wheel displacement is out of range.")
end