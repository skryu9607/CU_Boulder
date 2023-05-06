function PlotSimulation(time, aircraft_state_array,control_inputs_array,col)
% Input : the length n vector which holds the time corresponding to the set
% of variables., 12 by n array of aircraft state, the 4 by n array of
% control inputs, and the string "col" which indicates the plooting optinon
% used for every plot. 
% The function should create a total of 6 figures. 
% Position, Euler angles, Inertial velocity, angular velocity.
% One figure with four subplots for each control input variable
% 3D path of the aircraft, with positive heigh upward in the figure. 
% Indicate the start (green) and finish (red) of the path with different
% colored markers. 

% The function must be able to called multiple times for different
% simulation runs with dfferent col indicators. 
%close all;
figure;
hold on;
title("Position")
subplot(311);
plot(time, aircraft_state_array(1,:),col);
ylabel("X[m]")  
subplot(312);
plot(time, aircraft_state_array(2,:),col);
ylabel("Y[m]")  
subplot(313);
plot(time, -aircraft_state_array(3,:),col);
ylabel("Z[m]");hold off
figure;
hold on;
title("Euler angles")
subplot(311);
plot(time, rad2deg(aircraft_state_array(4,:)),col);
ylabel("Phi[deg]")
subplot(312);
plot(time, rad2deg(aircraft_state_array(5,:)),col);
ylabel("Tha[deg]")
subplot(313);
plot(time, rad2deg(aircraft_state_array(6,:)),col);
ylabel("Psi[deg]");hold off;

figure;
hold on;
title("Inertial Velocity")
subplot(311);
plot(time, aircraft_state_array(7,:),col);
ylabel("u[m/s]")
hold on;
subplot(312);
plot(time, aircraft_state_array(8,:),col);
ylabel("v[m/s]")
subplot(313);
plot(time, aircraft_state_array(9,:),col);
ylabel("w[m/s]");hold off;

figure;
hold on;
title("Angular velocity")
subplot(311);
plot(time, aircraft_state_array(10,:),col);
ylabel("p[rad/s],roll rate");
subplot(312);
plot(time, aircraft_state_array(11,:),col);ylabel("q[rad/s],pitch rate");
subplot(313);
plot(time, aircraft_state_array(12,:),col);ylabel("r[rad/s],yaw rate");hold off

figure;
hold on;
title("Control INPUTs")
subplot(411);
plot(time,control_inputs_array(1,:));ylabel("Elevator");
subplot(412);
plot(time,control_inputs_array(2,:));ylabel("Aileron");
subplot(413);
plot(time,control_inputs_array(3,:));ylabel("Rudder");
subplot(414);
plot(time,control_inputs_array(4,:));ylabel("Throttle");hold off

figure;
X = aircraft_state_array(1:3,:);
plot3(X(1,:),X(2,:),-X(3,:),col);hold on
plot3(X(1,1),X(2,1),-X(3,1),'gd')
plot3(X(1,end),X(2,end),-X(3,end),'rd');hold off

end

