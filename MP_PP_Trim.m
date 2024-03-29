% Seung-Keol Ryu
% created : 3/13/2023
%% "Life is short, and art is long."

%% The code of Motion primitives based path planners
clc;clear all;close all;
run("ttwistor.m")
%% Make the initial setting (usually constant)
ap = aircraft_parameters;
global va wind_inertial goal
wind_inertial = [-.5,-.5,-1]'; % Inertial velocity by body frame.
va = 25;Ts = 20;eps = 100;

%% Make the start and goal states
% States : [Pn,Pe,chi,gamma,h]
x0 = [0,0,deg2rad(0),deg2rad(0),-2000]';
goal = [350,350,0,0,-2900];

% %% Make the Thermals objects
% thm1 = thmals;
% % add(obj,x,y,z,radius)
% thm1 = add(thm1,+400,+600,-2300,300);
% wd_in_th = [0,0,-10];
% thm1 = wd(thm1,wd_in_th);
% 
% %% Make the Obstacle objects
% obs1 = obstacles;
% obs1 = add(obs1,600,0,-2200,300);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Make the Inputs for all elements of Motion Primitives.

% u2 = 0, u3 = 0 : Straight motion
% Straight motion : u2 = 0, u3 = 0,
% u2 ~= 0, u3 ~= 0 : Turn.

u2_cand = [deg2rad(30/Ts),deg2rad(-30/Ts),deg2rad(90/Ts),deg2rad(-90/Ts),0];
%u2_cand = u2_cand/2;
u3_cand =[deg2rad(10),deg2rad(-10),0];
%u3_cand =[deg2rad(5),deg2rad(-5),deg2rad(10),deg2rad(-10),0];
%u3_cand = u3_cand/2;

% course angle, flight path angle, u2, u3
rsl = 1;
MP_Inputs = All_MPs(Ts,rsl,'VIS');

%% STEP 1. Making the tree expanded by Motion primitives.
tic
num = 0;
str_MP = MP_Inputs{1};
cuv_MP = MP_Inputs{2};
spr_MP = MP_Inputs{3};
spl_MP = MP_Inputs{4};
n = length(str_MP) + length(cuv_MP) + length(spr_MP) + length(spl_MP);
thm_number = 0;

% Make the nodes of a tree.
mynodes = node;
mynodes.S = [0];
mynodes.T = [1];
mynodes = addNode(mynodes,x0(1),x0(2),x0(3),x0(4),x0(5));
previous_nodes = [x0];
new_nodes = [];
mynodes.mp = [0];
mynodes = init_hc(mynodes,1);
norm_L = norm([x0(1),x0(2),x0(5)]-[goal(1),goal(2),goal(5)]',2);
mynodes.hrsts = norm_L;

mynodes.costs = 0;
% Collision check.
col_idx = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Expansion is starting.
% 4 depth, Too dense
% 3 depth, 
full_cand_idx = zeros(n,1);
depth = 4; best = 10; % Best 8 nodes.
% set the depth of a tree.
while num < depth
    num = num + 1;
    % k is an element of the previous nodes.
    for kk = 1:size(previous_nodes,2)
        
        % i is an indicator of MP_Inputs
        for i = 1:n
            % Assign a single set of Motion primitives.
            MP_Input = single_MP(MP_Inputs,i);
            % Set the x_old to a certain element of a set of previous_nodes.
            x_old = previous_nodes(:,kk);
            % Just indication of locations of a point.
            pt0 = [x_old(1),x_old(2),x_old(5)]';
            % Effect of i-th motion primitive to x_old.
            x_del = MP(x_old,Ts,MP_Input);
            % Normally, wind_effect is just Ts*constant wind_inertial.
            wind_effect = Ts * [wind_inertial(1),wind_inertial(2),0,0,wind_inertial(3)]';
            x1 = x_old + x_del' + wind_effect;
            pt1 = [x1(1),x1(2),x1(5)]';

            % Check the thermal effects of x0 : start point and x1 : end
            % point. Flag = 1 : collision, Flag = 0 : not collide with
            flag_x0_thm1 = collide(pt0,thm1.source,thm1.radius,1);
            flag_x1_thm1 = collide(pt1,thm1.source,thm1.radius,1);

            % i-th Motion Primitives.
            MotionPrim = MP_Input;

            % End point in the influence of thermals

%             if flag_x1_thm1
% 
%                 % Where is the collision point?
%                 delt = intpl(x_old,thm1,MotionPrim,Ts);
% 
%                 % Using interpolation, find the collision point and update
%                 % the wind_effects.
% 
%                 wind_effect = delt * wind_inertial + (Ts-delt) * thm1.wind;
%                 wind_effect = [wind_effect(1),wind_effect(2),0,0,wind_effect(3)]';
%                 x1 = x_old + x_del' + wind_effect;
%                 pt1 = [x1(1),x1(2),x1(5)]';
% 
%                 thm_number = thm_number + 1;
% 
%                 % Start point in the influence of thermals
%             elseif flag_x0_thm1
% 
%                 delt = intpl(x1,thm1,-MotionPrim,Ts);
%                 wind_effect = delt * thm1.wind + (Ts-delt) * wind_inertial;
%                 wind_effect = [wind_effect(1),wind_effect(2),0,0,wind_effect(3)]';
%                 x1 = x_old + x_del' + wind_effect;
%                 pt1 = [x1(1),x1(2),x1(5)]';
% 
%                 thm_number = thm_number + 1;
% 
%             end
            % Check the collision with obstacle(s).

            flag_obs1 = collide(pt1,obs1.source,obs1.radius,0);

            % Connect the expanded nodes and the previous nodes.

            new_nodes(:,n*(kk-1)+i) = x1;
            mynodes = addNode(mynodes,x1(1),x1(2),x1(3),x1(4),x1(5));
            mynodes.mp = [mynodes.mp,i];
            if num == 1

                mynodes = connectNodes(mynodes,1,1+i);
                if flag_obs1
                    col_idx = [col_idx,1+i];
                end
            else
                if flag_obs1
                    col_idx = [col_idx,(n^(num)-1)/(n-1) + n*(kk-1)+i];
                end
                % The step 2 will use the collide_idx to check the feasibility of trajectories.
                mynodes = connectNodes(mynodes,(n^(num-1)-1)/(n-1)+kk,(n^(num)-1)/(n-1)+n*(kk-1)+i);

            end
            % 특정 노드로부터 나오는 cand_idx인데, absolute index를 가짐.
            full_cand_idx(i) = (best^(num)-1)/(best-1)+best*(kk-1)+i;
        end
        %prev_index = (best^(num-1)-1)/(best-1)+kk;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Make all candidates evaluated.
        % 비교할 것들. full_cand_idx.
        % It's time to evaluate the heruistics and costs
        % choose the best "# of best" motion primitives. 
        % 하나의 stops에서 나온 all motion primitives를 비교분석.
        mynodes = assign_hrst_cost(MP_Inputs,mynodes,full_cand_idx,norm_L);
        alpha = 0.0;
        eval_func = alpha * mynodes.hrsts(full_cand_idx) - (1-alpha) * mynodes.costs(full_cand_idx);
        
        [B,I] = sort(eval_func,'ascend');  %Ascending.
        % Real value by Relative index
        % Absolute Index
        BEST  = B(1:best);
        Best_mp_idx = I(1:best); % These are relative indeces.
        Best_idx = full_cand_idx(Best_mp_idx);
        Remove = setdiff(full_cand_idx,Best_idx);
        % Relative Index
        
        % Saveing which motion primitives.
        for i = 1:length(Best_mp_idx)
            mynodes.mp(Best_idx(i)) = Best_mp_idx(i);
        end

        % Pruning.
        for i = length(Remove):-1:1
            mynodes = DeleteNodes(mynodes,Remove(i));
        end 
        % Collect the best branches
    end
    STOPS = mynodes.sts(:,end-(best^(num)-1):end);
    % Updates.
    previous_nodes = STOPS;
    % Replicate the previous nodes ,and those would be vacant space of the
    % next expanded nodes.
    new_nodes = repmat(STOPS,1,n);
end
toc
%% Plot - Step 1
figure;

plot3(mynodes.sts(1,:),mynodes.sts(2,:),-mynodes.sts(5,:),'bo','markersize',10)
hold on;
%plot3(mynodes.sts(1,col_idx),mynodes.sts(2,col_idx),-mynodes.sts(5,col_idx),'go','markersize',6,'markerfacecolor','g')

xlabel('x[m]');ylabel('y[m]');zlabel('h[m]');
title("MP-PP with obstacles and thermals,A Full graph of nodes ")
grid on;
plot3(x0(1),x0(2),-x0(5),'ko','markersize',8,'markerfacecolor','k');
plot3(goal(1),goal(2),-goal(5),'ro','markersize',8,'markerfacecolor','r');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Make the reached indices.

rch_nodes = [];rch_nodes_idx = [];

for i = 1 : length(mynodes.sts)
    
    flag = collide(mynodes.pts(:,i),[goal(1),goal(2),goal(5)]',eps,0);
    
    if flag
        rch_nodes_idx = [rch_nodes_idx,i];
    end
    
end

rch_nodes = mynodes.pts(:,rch_nodes_idx);

plot3(rch_nodes(1,:),rch_nodes(2,:),-rch_nodes(3,:),'yo','markersize',8,'markerfacecolor','y')

legend("candidate nodes","The nodes near goal","A goal","Start",'location','best');

%% Make the trajectory and plot the trajectory.
% Even if the length of trajectory is different, the function Trajectory
% should work. 

% stops : the nodes found in step 1
% trajectory : a series of many nodes linking between stops. It looks like
% a continous curve if the number of nodes is large.
stops = [];prev = zeros(1,4);

for i = 1 : size(rch_nodes,2)
    
	STOP = Make_stops(rch_nodes_idx(i),mynodes);
    
    stops(i,:) = STOP;
%     if size(TRAJ,2) ~= size(prev,2)
%         Trajectory(1:i-1,size(prev,2)) = 0;
%         Trajectory(i,:) = TRAJ;
%     else
%         
%         Trajectory = [Trajectory;TRAJ;
%     end
%     prev = TRAJ;

end

% Specify reduced x of all elements.

x = mynodes.pts(1,:);
y = mynodes.pts(2,:);
z = mynodes.pts(3,:);

chi = mynodes.sts(3,:);
gam = mynodes.sts(4,:);

for i = 1 : size(stops,1)
    points = stops(i,:);
    
    plot3(x(points),y(points),-z(points),':om','markerfacecolor','m','LineWidth',2)  
end

hold off;
%% OUTPUT DISPLAY

%disp('The Trajectories are',num2str(Trajectory))
fprintf('The number of trajectories is %d \n',size(stops,1))
fprintf('The number of crashed nodes with obstacles is %d \n',size(col_idx,2))
fprintf('The number of influenced nodes with thermal updrafts is %d \n',thm_number)


%% Step 2. Feasibility steps
% The purpose of step 2 is check the feasibility of each trajectory from
% the first step. 1) Visualize and derive a more realistic curve from using
% stops. 2) I use the 
disp("Step 2")
% Firstly, Show the trajectories in step 1.
% Derive the errors.
% Between big nodes, separate the regions is about 20. -> calculate the trim condition. 
stops_red = [];

for i = 1:size(stops,1)
    
    for j = 1:size(stops,2)
        
        stops_red(:,j,i) = mynodes.sts(:,stops(i,j));
    
    end
    
end

% Derive each motion primitives between the nodes.
used_mp_map = derive_mp(stops,length(MP_Inputs));
% For more analysis, I need to make the function of showing x,y,gam,chi,h.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% A more precise trajectory
res_num = 1000;sml_t = Ts/res_num;
l =  size(stops,1); % In this case, the size of stops is only nine * (depth + 1).
Trajectories = cell(l);Trjt = [];

for i = 1:l
    % i : a series of stops : the ingredients of trajectories.
    for j = 1:size(stops,2)-1
        % j : an each stops of a certain i-th trajectories
        
        stops_old = stops_red(:,j,i);
        MP_each_stops = used_mp_map(i,j);
        
        for k = 1:res_num
            Trjt = [Trjt,stops_old];
            x_del = MP(stops_old,sml_t,MP_Inputs(MP_each_stops,:));
            wind_effect = sml_t * [wind_inertial(1),wind_inertial(2),0,0,wind_inertial(3)]';
            x1 = stops_old + x_del' + wind_effect;
            stops_old = x1;
        end
    end
    
    Trajectories{i} = Trjt;
end

figure;

title("A more precise trajectories")
hold on;grid on;view(3);

for i = 1:l
    plot3(Trajectories{i}(1,:)',Trajectories{i}(2,:)',-Trajectories{i}(5,:)','b')
    plot3(stops_red(1,:,i)',stops_red(2,:,i)',-stops_red(5,:,i)','ro')
end


%%
% 1. tra_red and used_mp_map -> trim_defi. : 
% ACS = ACS(MP) <- Make 
% Trim definition : [va,gamma0,height0,R0]

% Trim condition

% Try to implement things.
init = 0; final = depth * Ts; itr = res_num; n = depth*(itr+1);
time =  linspace(init,final,n);
dt = (final-init) / (n-1);
%time_reduced = linspace(init,Ts,4002);
as_all_array = [];acs_all_array = [];comparing_points = [];
%for k = 1:size(used_mp_map,1)
j = 1;k = 1;

MP_a_tra = used_mp_map(k,j);
U = MP_Inputs(MP_a_tra,:);
x_r = mynodes.sts(:,stops(k,j));
gamma0 = U(2);

xtd0 = [va,gamma0,-x_r(5),va/U(1)]';
while j < depth+1
% Refine the simulation full step 1 path and Compare step 2 
    MP_a_tra = used_mp_map(k,j);gamma0 = U(2);
    U = MP_Inputs(MP_a_tra,:)
    
    xtd0 = [va,gamma0,-x_r(5),va*cos(gamma0)/U(1)]';
    x_r = mynodes.sts(:,stops(k,j));
    
    [as,acs] = coor_min_trim_fun(xtd0,aircraft_parameters);
    as(1) = x_r(1);as(2) = x_r(2);
    aircraft_state_array = [as];control_inputs_array = [acs];
    
    for i = 1:itr
        % Calculating trim is already written.
        [xdot] = AC_EOM(time,as,acs,wind_inertial,aircraft_parameters);
        aircraft_new_state = as + dt * xdot;
        aircraft_state_array = [aircraft_state_array,aircraft_new_state];
        control_inputs_array = [control_inputs_array,acs];
        as = aircraft_new_state;
    end
    % U is Motion Primitives.
    % [the rate of course angle(nearly heading angle), flight path angle]
    %x_r = [as(1),as(2),as(6),U(2),as(3)];
    j = j + 1;
    comparing_points = [comparing_points,as];
    as_all_array = [as_all_array,aircraft_state_array];
    acs_all_array = [acs_all_array,control_inputs_array];
    disp("The update is done")



    %     xtd0 = [va,gamma0,-x_r(5),va/U(1)]';
%     [as,acs] = min_trim_fun(xtd0,aircraft_parameters);
%     as(1) = x_r(1);as(2) = x_r(2); as(3) = x_r(5);
%     aircraft_state_array = [as];control_inputs_array = [acs];
%     for i = 1:itr
%         % Calculating trim is already written.
%         [xdot] = AC_EOM(time,as,acs,wind_inertial,aircraft_parameters);
%         aircraft_new_state = as + dt * xdot;
%         aircraft_state_array = [aircraft_state_array,aircraft_new_state];
%         control_inputs_array = [control_inputs_array,acs];
%         as = aircraft_new_state;
%     end
%     comparing_points = [comparing_points,as];
%     as_all_array = [as_all_array,aircraft_state_array];
%     acs_all_array = [acs_all_array,control_inputs_array];
%     disp("The update is done")
%     j = j + 1;
end

%end
disp("The update is done")
col = 'bo';
%PlotSimulation(time, as_all_array,acs_all_array,col)
pos_rdc_EOM = [[x0(1),x0(2),x0(5)]',comparing_points(1:3,:)];
error = pos_rdc_EOM - mynodes.pts(:,stops(k,:))
error = mean(abs(error))
toc
% %% Step 3. Cost evaluations : the size of acs values
% 


%%
