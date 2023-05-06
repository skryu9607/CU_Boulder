clc;clear all;close all;
run("ttwistor.m")
ap = aircraft_parameters;
%% Start the simulation!!!!
% Not trim.
% 1) Straight without height change
% trim value -> Trim values. motions itself the same. 
% 2) Turn (Level-flight)
% 3) Climb 
% 4) Descent
% 5) Turn with climb
% wind_inertial = [1,1,-10]';
% goal = [100,100,400];
% eps = 30; % 3/4. 
% 
% % SIX variables.
% % X = [0,0,100,va,deg2rad(0),deg2rad(0)];
x = [0,0,50]';va = 40;
init = 0; final = 15; 
dt = 0.01;
n = (final-init) / dt + 1;
time =  linspace(init,final,n);
initial_state = initi
nodes = Traj(x,MP_strh(va,wind_inertial),5);
plot3(nodes(1,:),nodes(2,:),nodes(3,:),'ro')

% % plot3(MP.straight(:,1),MP.straight(:,2),MP.straight(:,3),'ro');
% % hold on;
% % plot3(MP.turn_climb(:,8),MP.turn_climb(:,8),MP.turn_climb(:,3));grid on;
% 
% % 2. Make the graph
% % Start,Goal
% % va_t = sqrt(sum(as(7:9).^2));
% % start = as;
% % x = start;
% % X = [x];
% % x = [0,0,100,
% % for i = 1:n
% %     dt = 0.01;
% %     time = 0;
% %     X = [X,x];
% %     xdot(3) = -va_t * sin(deg2rad(-5));
% %     xdot(4) = 0;
% %     xdot(5) = 0;
% %     xdot(7:12) = 0;
% %     xdot(6) = 1 * va_t/100;
% %     x = x + dt * xdot;
% % end
% % grid on
% % x_goal = [0,100];y_goal = [0,100];z_goal = [100,200];
% % %open_list = [start,start+[1,1,10]'];
% % plot3(x_goal,y_goal,z_goal,'ro')
% % hold on;
% % [x(1),x(2),-x(3)]
% % plot3(x(1),x(2),-x(3),'ro')
% % plot3(X(1,:),X(2,:),-X(3,:),'r')
% % xlabel('x');ylabel('y');zlabel('z');
% %motions = MotionPrimitives(AC.m,AC.S);
%%
wind_inertial = [1,1,1]';
va = 20;
X0 = [0,0,0,0,100]';X = X0;
u2 = deg2rad(1);
u3 = deg2rad(2);
for i = 1:2:length(inputs)-1
    u2 = inputs(i)
    u3 = inputs(i+1)
    X = MP(X,Ts,u2,u3,wind_inertial);
end





