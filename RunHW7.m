% Eric W. Frew
% ASEN 5519
% RunHW6.m
% Created: 3/2/23
%  
% This is a helper that students can use to complete HW 7. 
%
%

close all;% <========= Comment out this line and you can run this file multiple times and plot results together
clear all; 



%%% Define parameters for specifying control law
SLC = 2;
FEED = 1;


%%% Set flags
ANIMATE_FLAG = 0; % <========= Set to 1 to show animation after simulation
CONTROL_FLAG = FEED; % <========= Set to control law to use (SLC or FEED)


%%% Aircraft parameter file
ttwistor


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Determine trim state and control inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

V_trim = 18;
h_trim = 1805;
gamma_trim = 0;
trim_definition = [V_trim; gamma_trim; h_trim];


%%% STUDENTS REPLACE THESE TWO FUNCTIONS WITH YOUR VERSIONS FROM HW3/4
[trim_variables, fval] = CalculateTrimVariables(trim_definition, aircraft_parameters);
[aircraft_state_trim, control_input_trim] = TrimConditionFromDefinitionAndVariables(trim_variables, trim_definition);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Determine control gains
%%% See 'RunGVF.m for how gain files were created
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


if (CONTROL_FLAG==FEED)
    gains_file = 'ttwistor_gains_feed';
    fprintf(1, '\n==================================== \nAUTOPILOT: SLC with Feedforward\n \n')
else
    gains_file = 'ttwistor_gains_slc';
    fprintf(1, '\n ==================================== \nAUTOPILOT: Simple SLC\n \n')
end

load(gains_file)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Set input commands for autopilot.
%%%
%%% Note, STUDENTS may need to change these while tuning the autopilot.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

orbit_speed = 18;
orbit_radius = 200;
orbit_center = [300;300;-1805];
orbit_flag=1;
orbit_gains.kr = .01;
orbit_gains.kz = .001;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Set aircraft and simulation initial conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aircraft_state0 = aircraft_state_trim;

aircraft_state0(3,1) = -1655; %<------- CLIMB mode starts when aircraft reaches h = 1675
aircraft_state0(4,1) = 0*pi/180; %180*pi/180; %<------- WHEN CONFIDENT INITIALIZE UPSIDE DOWN!!

control_input0 = control_input_trim;

wind_inertial = [0;0;0];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Set simulation and control parameters
%%%
%%% Note, the simulation runs on two separate times scales. The variable Ts
%%% specifies the "sample time" of the control system. The control law
%%% calculates a new control input every Ts seconds, and then holds that
%%% control input constant for a short simulation of duration Ts. Then, a
%%% new control input is calculated and a new simulation is run using the
%%% output of the previous iteration as initial condition of the next
%%% iteration. The end result of each short simulation is stored as the
%%% state and control output. Hence, the final result is a simulation with
%%% state and control input at every Ts seconds.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Ts = .1;
Tfinal = 450;
control_gain_struct.Ts=Ts;

%%% iterate at control sample time
n_ind = Tfinal/Ts;

aircraft_array(:,1) = aircraft_state0;
control_array(:,1) = control_input0;
time_iter(1) = 0;


for i=1:n_ind

    TSPAN = Ts*[i-1 i];

    wind_array(:,i) = wind_inertial;

    wind_body = TransformFromInertialToBody(wind_inertial, aircraft_array(4:6,i));
    air_rel_vel_body = aircraft_array(7:9,i) - wind_body;
    wind_angles(:,i) = WindAnglesFromVelocityBody(air_rel_vel_body);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Guidance level commands
    %%%
    %%%   control_objectives(1) = h_c;
    %%%   control_objectives(2) = h_dot_c;
    %%%   control_objectives(3) = chi_c;
    %%%   control_objectives(4) = chi_dot_ff;
    %%%   control_objectives(5) = Va_c;
    %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % STUDENTS WRITE THIS FUNCTION
    % control_objectives = OrbitGuidance(aircraft_array(1:3,i), orbit_speed, orbit_radius, orbit_center, orbit_flag, orbit_gains);
    control_objectives = [100; 0; 45*pi/180; 0; V_trim]; %<============== Comment out when OrbitGuidance is complete

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autopilot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if (CONTROL_FLAG==FEED)
        [control_out, x_c_out] = SLCWithFeedForwardAutopilot(Ts*(i-1), aircraft_array(:,i), wind_angles(:,i), control_objectives, control_gain_struct);
    else
        [control_out, x_c_out] = SimpleSLCAutopilot(Ts*(i-1), aircraft_array(:,i), wind_angles(:,i), control_objectives, control_gain_struct);
    end
    
    control_array(:,i) = control_out;
    x_command(:,i) = x_c_out;
    x_command(5,i) = trim_variables(1);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Aircraft dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [TOUT2,YOUT2] = ode45(@(t,y) AircraftEOM(t,y,control_array(:,i),wind_inertial,aircraft_parameters),TSPAN,aircraft_array(:,i),[]);


    aircraft_array(:,i+1) = YOUT2(end,:)';
    time_iter(i+1) = TOUT2(end);
    wind_array(:,i+1) = wind_inertial;
    control_array(:,i+1) = control_array(:,i);
    x_command(:,i+1) = x_command(:,i);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%PlotSimulation(time_iter,aircraft_array,control_array, wind_array,'b')
PlotSimulationWithCommands(time_iter,aircraft_array,control_array, wind_array, x_command, 'b')

for c = 0:1:360;
    circ_orbit(:,c+1) = orbit_center + orbit_radius*[cosd(c); sind(c); 0];
end
figure(8);
plot3(circ_orbit(1,:),circ_orbit(2,:),-circ_orbit(3,:),'k:');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Animate aircraft flight
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (ANIMATE_FLAG)
    pause();
    %DefineDefaultAircraft
    DefineTTwistor

    for aa = 1:length(time_iter)
        DrawAircraft(time_iter(aa), aircraft_array(:,aa), pts);
    end

    AnimateSimulation(time_iter, aircraft_array')
end
