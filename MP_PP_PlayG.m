% Seung-Keol Ryu
% created : 3/13/2023
%% "Life is short, and art is long."

%% The code of Motion primitives based path planners
clc;clear all;close all;
ttwistor;
%% Make the initial setting (usually constant)
ap = aircraft_parameters;
global wind_inertial goal
wind_inertial = [-.5,-.5,-1]'; % Inertial velocity by body frame.
Ts = 10;eps = 100;
% u1 = va;
va_cand = [17,22];col ='r';
%% Make the start and goal states
% States : [Pn,Pe,chi,h]
x0 = [0,0,deg2rad(0),-1655]';
%goal = [300,600,0,0,-2400];
goal = [300,-500,0,-2000];  %depth = 3, eps = 100;
%% Aug 22th
% state 차원 줄이기 (check!)
% thermals and objects랑 start랑 goal만 있는 맵 만들기 (check!)
% costs 계산하기 (accumulative하게)
% Bi-directional search하기

%% Make the Thermals objects
thm1 = thmals;
% add(obj,x,y,z,radius)
draw_num = 100;
center_th = [200,-100,-2100];R_thm = 50;wd_th = [0,0,-15];
thm1 = add(thm1,center_th,R_thm,wd_th);
[x_t,y_t,z_t] = cylinder(R_thm,draw_num);
z_t = z_t * (goal(end)-x0(end)) + x0(end);
x_t = x_t + center_th(1);
y_t = y_t + center_th(2);


%% Make the Obstacle objects
obs1 = obstacles;
center_ob = [200,-500,-1900];
R_ob = 50;
obs1 = add(obs1,center_ob,R_ob);
[x_o,y_o,z_o] = sphere(draw_num);
x_o = x_o*R_ob+center_ob(1);
y_o = y_o*R_ob+center_ob(2);
z_o = z_o*R_ob+center_ob(3);

hold on;
surf(x_t,y_t,-z_t)
grid on;
plot3(x_o,y_o,-z_o,'go')
plot3(x0(1),x0(2),-x0(end),'ro');plot3(goal(1),goal(2),-goal(end),'ko');
view(3);axis equal;xlabel('x[m]');ylabel('y[m]');zlabel('z[m]');
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Make the Inputs for all elements of Motion Primitives.
% u1 : air relative velocity
% u2 = 0, u3 = 0 : Straight motion
% Straight motion : u2 = 0, u3 = 0,
% u2 ~= 0, u3 ~= 0 : Turn.
% course angle, flight path angle, u2, u3
rsl = 1;
MP_Inputs = All_MPs(Ts,rsl,1);

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
mynodes = addNode(mynodes,x0(1),x0(2),x0(3),x0(4));
previous_nodes = [x0];
new_nodes = [];
mynodes.mp = [0];
mynodes = init_hc(mynodes,1);
norm_L = norm([x0(1),x0(2),x0(end)]-[goal(1),goal(2),goal(end)]',2);
mynodes.hrsts = norm_L;

mynodes.costs = 0;
% Collision check.
col_idx = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Expansion is starting.
% 4 depth, Too dense
% 3 depth, 
full_cand_idx = zeros(n,1);
depth = 5; best = 10; % Best 8 nodes.
% set the depth of a tree.

% They 
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
            pt0 = [x_old(1),x_old(2),x_old(end)]';
            % Effect of i-th motion primitive to x_old.
            x_del = MP(x_old,Ts,MP_Input);
            % Normally, wind_effect is just Ts*constant wind_inertial.
            wind_effect = Ts * [wind_inertial(1),wind_inertial(2),0,wind_inertial(3)]';
            x1 = x_old + x_del' + wind_effect;
            pt1 = [x1(1),x1(2),x1(end)]';

            % Check the thermal effects of x0 : start point and x1 : end
            % point. Flag = 1 : collision, Flag = 0 : not collide with
            flag_x0_thm1 = collide(pt0,thm1.source,thm1.radius,1);
            flag_x1_thm1 = collide(pt1,thm1.source,thm1.radius,1);

            % i-th Motion Primitives.
            MotionPrim = MP_Input;

            % End point in the influence of thermals

            if flag_x1_thm1

                % Where is the collision point?
                delt = intpl(x_old,thm1,MotionPrim,Ts);

                % Using interpolation, find the collision point and update
                % the wind_effects.

                wind_effect = delt * wind_inertial + (Ts-delt) * thm1.wind;
                wind_effect = [wind_effect(1),wind_effect(2),0,wind_effect(3)]';
                x1 = x_old + x_del' + wind_effect;
                pt1 = [x1(1),x1(2),x1(end)]';

                thm_number = thm_number + 1;

                % Start point in the influence of thermals
            elseif flag_x0_thm1
                if isa(MP_Input,"cell")
                    MotionPrim{1} = -MotionPrim{1};
                    MotionPrim{2} = -MotionPrim{2};
                else
                    MotionPrim = -MotionPrim;
                end
                delt = intpl(x1,thm1,MotionPrim,Ts);
                wind_effect = delt * thm1.wind + (Ts-delt) * wind_inertial;
                wind_effect = [wind_effect(1),wind_effect(2),0,wind_effect(3)]';
                x1 = x_old + x_del' + wind_effect;
                pt1 = [x1(1),x1(2),x1(end)]';

                thm_number = thm_number + 1;

            end
            % Check the collision with obstacle(s).

            flag_obs1 = collide(pt1,obs1.source,obs1.radius,0);
            % Connect the expanded nodes and the previous nodes.

            new_nodes(:,n*(kk-1)+i) = x1;
            mynodes = addNode(mynodes,x1(1),x1(2),x1(3),x1(end));
            Instant = addNode(Instant,x1(1),x1(2),x1(3),x1(end));
            mynodes.mp = [mynodes.mp,i];
            % collision check.
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
                mynodes = connectNodes(mynodes,(n^(num-1)-1)/(n-1)+kk+1 ,(n^(num)-1)/(n-1)+n*(kk-1)+i);

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
        Instant = node;
        Instant.S = [(best^(num-1)-1)/(best-1) + kk];
        Instant.sts = x_old;
        mynodes = update_hrst_cost(MP_Inputs,mynodes,full_cand_idx,norm_L);
        alpha = 0.5;
        eval_func = (1-alpha) * mynodes.hrsts(full_cand_idx) + alpha * mynodes.costs(full_cand_idx);
        [B,I] = sort(eval_func,'Descend');  %Ascending.
        % Real value by Relative index
        % Absolute Index
        BEST  = B(1:best);
        Best_mp_idx = I(1:best); % These are relative indeces.
        Best_idx = full_cand_idx(Best_mp_idx);
        
        Remove = setdiff(full_cand_idx,Best_idx);

        % Relative Index
        % Saving which motion primitives.
        for i = 1:length(Best_mp_idx)
            mynodes.mp(Best_idx(i)) = Best_mp_idx(i);
        end

        % Pruning.
        for i = length(Remove):-1:1
            mynodes = DeleteNodes(mynodes,Remove(i));
        end 
        
    end
    STOPS = mynodes.sts(:,end-(best^(num)-1):end);
    % Updates.
    previous_nodes = STOPS;
    % Replicate the previous nodes ,and those would be vacant space of the
    % next expanded nodes.
    if num == depth - 1
        disp("Break")
        break
    else
        new_nodes = repmat(STOPS,1,n);
        disp("Expand")
    end
end
% Updating
for i = 1:length(mynodes.T)
    if i == 1
        mynodes.T(1) = 1;
        mynodes.S(1) = 0;
    else
        mynodes.T(i) = i;
        %mynodes.S(i) = fix((i-2)/best)+1;
    end
end
disp("Plot")
%% Plot - Step 1
figure;

plot3(mynodes.sts(1,:),mynodes.sts(2,:),-mynodes.sts(end,:),'bo','markersize',10)
hold on;
%plot3(mynodes.sts(1,col_idx),mynodes.sts(2,col_idx),-mynodes.sts(5,col_idx),'go','markersize',6,'markerfacecolor','g')
fntsz = 20;xlabel("x [m]",'fontsize',fntsz);ylabel("y [m]",'fontsize',fntsz);zlabel("z [m]",'fontsize',fntsz);
title("MP-PP with obstacles and thermals,A Full graph of nodes ",'fontsize',fntsz)
grid on;
plot3(x0(1),x0(2),-x0(end),'ko','markersize',8,'markerfacecolor','k');
plot3(goal(1),goal(2),-goal(end),'ro','markersize',8,'markerfacecolor','r');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Make the reached indices.

% nodes means positions
rch_nodes = [];rch_nodes_idx = [];

for i = 1 : length(mynodes.sts)
    
    flag = collide(mynodes.pts(:,i),[goal(1),goal(2),goal(end)]',eps,0);
    
    if flag
        rch_nodes_idx = [rch_nodes_idx,i];
    end
    
end

rch_nodes = mynodes.pts(:,rch_nodes_idx);

plot3(rch_nodes(1,:),rch_nodes(2,:),-rch_nodes(3,:),'yo','markersize',8,'markerfacecolor','y')
%plot3(mynodes.sts(1,col_idx),mynodes.sts(2,col_idx),-mynodes.sts(end,col_idx),'go','markersize',6,'markerfacecolor','g')
%legend("candidate nodes","The nodes near goal","A goal","Start",'location','best');

%% Make the trajectory and plot the trajectory.
% Even if the length of trajectory is different, the function Trajectory
% should work. 

% stops : the nodes found in step 1
% trajectory : a series of many nodes linking between stops. It looks like
% a continous curve if the number of nodes is large.
stops = Make_stops(rch_nodes_idx,mynodes);
stops = stops';
STOPS = cell(length(stops),1);
for i = 1:length(stops)
    STOPS{i,1} = mynodes.pts(:,stops{i});
end
% Specify reduced x of all elements.

x = mynodes.pts(1,:);
y = mynodes.pts(2,:);
z = mynodes.pts(3,:);

chi = mynodes.sts(3,:); % NOT CHI DOT
gam = mynodes.sts(4,:);
%va = mynodes.sts(5,:);
for j = 1:size(stops,1) % Num of trjt
    for i = 1 : length(STOPS{j}) % lenght of each trjt
        full_dyn_pts = STOPS{j}(:,i);

        plot3(full_dyn_pts(1),full_dyn_pts(2),-full_dyn_pts(3),':om','markerfacecolor','m','LineWidth',4)  
    end
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
stops_red = cell(size(stops,1),1);

for i = 1:size(stops,1)
    stops_red{i} = mynodes.sts(:,stops{i});
end

str_MP = MP_Inputs{1};
cuv_MP = MP_Inputs{2};
spr_MP = MP_Inputs{3};
spl_MP = MP_Inputs{4};
NN = length(str_MP) + length(cuv_MP) + length(spr_MP) + length(spl_MP);
% Derive each motion primitives between the nodes.
%used_mp_map = derive_mp(stops,NN);
% For more analysis, I need to make the function of showing x,y,gam,chi,h.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% A more precise trajectory
res_num = 10000;sml_t = Ts/res_num;
l =  length(stops); % In this case, the size of stops is only nine * (depth + 1).
smp_path = cell(l,1);Trjt = [];

for i = 1:l
    % i : number of trajectories
    Trjt = [];
    for j = 1:length(stops{i})-1
        % j : an each stops of a certain i-th trajectories
        
        stops_old = stops_red{i}(:,j);
        MP_each_stops = mynodes.mp(stops{i}(j+1));
        sng_mp = single_MP(MP_Inputs,MP_each_stops);
        for k = 1:res_num
            Trjt = [Trjt,stops_old];
            x_del = MP(stops_old,sml_t,sng_mp);
            wind_effect = sml_t * [wind_inertial(1),wind_inertial(2),0,wind_inertial(3)]';
            x1 = stops_old + x_del' + wind_effect;
            stops_old = x1;
        end
    end
    smp_path{i} = Trjt;
end

figure;

title("A more precise trajectories",'fontsize',fntsz);
xlabel("x [m]",'fontsize',fntsz);ylabel("y [m]",'fontsize',fntsz);zlabel("z [m]",'fontsize',fntsz);
hold on;grid on;view(3);

for i = 1:l
    plot3(smp_path{i}(1,:)',smp_path{i}(2,:)',-smp_path{i}(end,:)','bo')
    plot3(stops_red{i}(1,:)',stops_red{i}(2,:)',-stops_red{i}(end,:)','ro')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Full-Dynamics
% 1. tra_red and used_mp_map -> trim_defi. : 
% ACS = ACS(MP) <- Make 
% Trim definition : [va,gamma0,height0,R0]

% Trim conditions

% Try to implement things.
init = 0; final = depth * Ts; itr = res_num; n = depth*(itr);
dt = (final-init) / (n-1); full_dyn_pts = cell(l,1);
as_all_array = cell(l,1);
acs_all_array = cell(l,1);
for i = 1:length(smp_path)
    time =  linspace(init,final,n);
    aircraft_state_array = [];control_inputs_array=[];
    PTS = [];head = 0;
    for j = 1:length(stops{i})-1
        crt_idx = stops{i}(j+1);
        % Straight
        U = single_MP(MP_Inputs,mynodes.mp(crt_idx));
        MP_INDEX = mynodes.mp(crt_idx);% [x,y,phi_dot,gamma,va,h]
        if j == 1
            x_r = mynodes.sts(:,1);
        end
        if MP_INDEX < length(str_MP) + 1
            disp("Straight")
            va = U(1);gam0 = U(3);h = -x_r(end);
            xtd0 = [va,gam0,h]';
            [as,acs] = min_trim_fun(xtd0,aircraft_parameters);
            Res=AirRelativeVelocityToWindAngles(as(7:9));
            as(1) = x_r(1);
            as(2) = x_r(2);
            as(4) = atan(U(2)*U(1)/ap.g);
            as(5) = gam0 + Res(3);
            as(6) = x_r(3);
            for k = 1:itr
                % Calculating trim is already written.
                [xdot] = AC_EOM(time,as,acs,wind_inertial,aircraft_parameters);
                aircraft_new_state = as + dt * xdot;
                aircraft_state_array = [aircraft_state_array,aircraft_new_state];
                control_inputs_array = [control_inputs_array,acs];
                as = aircraft_new_state;
                PTS = [PTS,[as(1),as(2),as(3)]'];
            end
            Res=AirRelativeVelocityToWindAngles(as(7:9));
            x_r = [as(1),as(2),as(6),as(3)]';
            head = as(6);
        
        % Spirals and curves    
        elseif MP_INDEX < length(str_MP) + length(cuv_MP) + length(spr_MP) + 1
            disp("Spirals and Curves")
            va = U(1);gam0 = U(3);h = -x_r(end);R = va/U(2);
            xtd0 = [va,gam0,h,R]' % gamma is constant.;
            [as,acs] = coor_min_trim_fun(xtd0,aircraft_parameters);
            Res=AirRelativeVelocityToWindAngles(as(7:9));
            as(1) = x_r(1);as(2) = x_r(2);as(4) = atan(U(2)*U(1)/ap.g);
            as(5) = gam0 + Res(3);as(6) = x_r(3);
            for k = 1:itr
                % Calculating trim is already written.
                [xdot] = AC_EOM(time,as,acs,wind_inertial,aircraft_parameters);
                aircraft_new_state = as + dt * xdot;
                aircraft_state_array = [aircraft_state_array,aircraft_new_state];
                control_inputs_array = [control_inputs_array,acs];
                as = aircraft_new_state;
                PTS = [PTS,[as(1),as(2),as(3)]'];
            end
            Res=AirRelativeVelocityToWindAngles(as(7:9));
            x_r = [as(1),as(2),as(6),as(3)]';
            head = as(6);
            
        % Splined curves
        else
            disp("Splined Curves")
            % U : cell.
            a = U{1}; % [x,y,phi_dot,gamma,va,h]
            va = a(1);gam0 = a(3);h = -x_r(end);R = va/a(2);
            xtd0 = [va,gam0,h,R]';
            [as,acs] = coor_min_trim_fun(xtd0,aircraft_parameters);
            Res=AirRelativeVelocityToWindAngles(as(7:9));
            as(1) = x_r(1);as(2) = x_r(2);as(4) = atan(a(2)*va/ap.g);
            as(5) = gam0 + Res(3);as(6) = x_r(3);

            for k = 1:itr/2
                % Calculating trim is already written.
                [xdot] = AC_EOM(time,as,acs,wind_inertial,aircraft_parameters);
                aircraft_new_state = as + dt * xdot;
                aircraft_state_array = [aircraft_state_array,aircraft_new_state];
                control_inputs_array = [control_inputs_array,acs];
                as = aircraft_new_state;
                PTS = [PTS,[as(1),as(2),as(3)]'];
            end
            x_r = [as(1),as(2),as(6),as(3)]';
            head = as(6);
            
            b = U{2};
            va = b(1);gam0 = b(3);h = -x_r(end);R = va/b(2);
            xtd0 = [va,gam0,h,R]';
            [as,acs] = coor_min_trim_fun(xtd0,aircraft_parameters);
            Res=AirRelativeVelocityToWindAngles(as(7:9));
            as(1) = x_r(1);as(2) = x_r(2);as(4) = atan(b(2)*va/ap.g);
            as(5) = gam0 + Res(3);as(6) = x_r(3);
            for k = itr/2+1:itr
                % Calculating trim is already written.
                [xdot] = AC_EOM(time,as,acs,wind_inertial,aircraft_parameters);
                aircraft_new_state = as + dt * xdot;
                aircraft_state_array = [aircraft_state_array,aircraft_new_state];
                control_inputs_array = [control_inputs_array,acs];
                as = aircraft_new_state;
                PTS = [PTS,[as(1),as(2),as(3)]'];
            end
            Res=AirRelativeVelocityToWindAngles(as(7:9));
            x_r = [as(1),as(2),as(6),as(3)]';
            head = as(6);
        end
    end
    full_dyn_pts{i} = PTS;
    as_all_array{i} = aircraft_state_array;
    acs_all_array{i} = control_inputs_array;
    disp("The update is done")
end
%%
figure;title("3D Comparision between full-dyn and simple-dyn",'fontsize',fntsz)
fntsz = 14;
xlabel("x [m]",'fontsize',fntsz);ylabel("y [m]",'fontsize',fntsz);zlabel("z [m]",'fontsize',fntsz);
grid on;
hold on;
for i = 1:length(stops)
    X = as_all_array{i};
    plot3(X(1,:),X(2,:),-X(3,:),col);
    plot3(X(1,end),X(2,end),-X(3,end),'rd','markersize',8,'markerfacecolor','r')
end

for i = 1:l
    plot3(smp_path{i}(1,:)',smp_path{i}(2,:)',-smp_path{i}(end,:)','b')
    plot3(stops_red{i}(1,:)',stops_red{i}(2,:)',-stops_red{i}(end,:)','ro','markersize',8,'markerfacecolor','r')
end
plot3(goal(1),goal(2),-goal(end),'bd','markersize',8,'markerfacecolor','b');
plot3(X(1,1),X(2,1),-X(3,1),'gd','markersize',8,'markerfacecolor','g')
view(3);
surf(x_t,y_t,-z_t);
grid on;
plot3(x_o,y_o,-z_o,'go');
%%
figure;title("2D Comparision between full-dyn and simple-dyn",'fontsize',fntsz)
fntsz = 14;
xlabel("x [m]",'fontsize',fntsz);ylabel("y [m]",'fontsize',fntsz);zlabel("z [m]",'fontsize',fntsz);
grid on;
hold on;
for i = 1:length(stops)
    X = as_all_array{i};
    plot3(X(1,:),X(2,:),-X(3,:),col);
    plot3(X(1,end),X(2,end),-X(3,end),'rd','markersize',8,'markerfacecolor','r')
end
for i = 1:l
    plot3(smp_path{i}(1,:)',smp_path{i}(2,:)',-smp_path{i}(end,:)','b')
    plot3(stops_red{i}(1,:)',stops_red{i}(2,:)',-stops_red{i}(end,:)','ro','markersize',8,'markerfacecolor','r')
end
plot3(goal(1),goal(2),-goal(end),'bd','markersize',8,'markerfacecolor','b');
plot3(X(1,1),X(2,1),-X(3,1),'gd','markersize',8,'markerfacecolor','g')
hold off
disp("The update is done")

%% Step 3. Cost evaluations : the size of acs values
smp_dyn_pts = cell(l,1);error = cell(l,1);
for i = 1:l
    smp_dyn_pts{i} = smp_path{i}([1,2,end],:);
    error{i} = mean(abs(full_dyn_pts{i} - smp_dyn_pts{i}),2);
end
%%
acs_costs = cell(l,1);acs_comp_costs = zeros(l,1);
for i  = 1:l
    acs_costs{i} = mean(abs(acs_all_array{i}),2);
end
for i = 1:l
    acs_comp_costs(i) = sum(acs_costs{i});
end
[value,best] = min(acs_comp_costs);
%%
figure;title("The solution",'fontsize',fntsz)
hold on;grid on;
i = best;X = as_all_array{best};
plot3(smp_path{i}(1,:)',smp_path{i}(2,:)',-smp_path{i}(end,:)','b')
plot3(stops_red{i}(1,:)',stops_red{i}(2,:)',-stops_red{i}(end,:)','ro','markersize',8,'markerfacecolor','r')
plot3(goal(1),goal(2),-goal(end),'bd','markersize',8,'MarkerFaceColor','b');
plot3(X(1,:),X(2,:),-X(3,:),col);
plot3(X(1,end),X(2,end),-X(3,end),'rd','markersize',8,'markerfacecolor','r')
plot3(x0(1),x0(2),-x0(end),'gd','markersize',8,'MarkerFaceColor','g')
view(3);
surf(x_t,y_t,-z_t);
grid on;
plot3(x_o,y_o,-z_o,'go');
%%
PlotSimulation(time, as_all_array{best},acs_all_array{best},col,goal)
