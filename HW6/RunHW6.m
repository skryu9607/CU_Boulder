% Eric W. Frew
% ASEN 5519
% RunHW6.m
% Created: 3/2/23
%  
% This is a helper that students can use to complete HW 6. 
%
%

close all;
clear all;

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

%[trim_variables, fval] = Trim_Expression(trim_definition, aircraft_parameters);
[aircraft_state_trim, control_input_trim] = min_trim_fun(trim_definition, aircraft_parameters);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Determine control gains
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% STUDENTS MUST COMPLETE THIS FUNCTION. A SKELETON OF THE FUNCTION IS PROVIDED

[control_gain_struct, linear_terms] = CalculateControlGainsSimpleSLC_Nondim_Ttwistor(aircraft_parameters, trim_definition, trim_variables)
control_gain_struct.u_trim = control_input_trim;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Set input commands for autopilot.
%%%
%%% Note, STUDENTS may need to change these while tuning the autopilot.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

h_c         = h_trim;  % commanded altitude (m)
h_dot_c     = 0;  % commanded altitude rate (m)
chi_c       = 40*pi/180;  % commanded course (rad)
chi_dot_ff  = 0;  % commanded course rate (rad)   
Va_c        = V_trim;  % commanded airspeed (m/s)
    


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
Tfinal = 500;
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
    %%% Note, the format is to allow flexibility for future assignments
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    control_objectives(1) = h_c;
    control_objectives(2) = h_dot_c;
    control_objectives(3) = chi_c;
    control_objectives(4) = chi_dot_ff;
    control_objectives(5) = Va_c;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Autopilot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [control_slc, x_c_slc] = SimpleSLCAutopilot(Ts*(i-1), aircraft_array(:,i), wind_angles(:,i), control_objectives, control_gain_struct);

    control_array(:,i) = control_slc;
    x_command(:,i) = x_c_slc;
    x_command(5,i) = trim_variables(1);
    %x_est(:,i) = zeros(16,1);


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
PlotSimulationWithCommands(time_iter,aircraft_array,control_array, wind_array, x_command, 'b')

%PlotSimulation(time_iter,aircraft_array,control_array, wind_array,'b')

