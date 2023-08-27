function [MP_Inputs] = All_MPs(Ts,rsl,flag)
n_set = 4;
MP_Inputs = cell(n_set,1);

wind_inertial = [0,0,0]';
% u1 : air relative velocity
u1_cand = [17,22]
% Ts = 20;
% rsl = 20;
sml_t = Ts/rsl
% No matter how complex the motion primitives are, the objective of the
% The First step of my research is making fancy curves. 
% I will make 
% 1. A straight line 2. Long curve 3. Spirals 4. Splined curves.
%% 1. A straight line
% A straight line can be five. 

u3_cand_1 = [deg2rad(45),deg2rad(15),deg2rad(0),deg2rad(-15),deg2rad(-45)];
N1 = length(u1_cand) * length(u3_cand_1);
MP_Inputs{1} = cell(N1,1);
for j = 1:length(u1_cand)
    u1 = u1_cand(j) * ones(rsl,1);
    for i = 1:length(u3_cand_1)

        u2 = zeros(rsl,1);
        u3 = u3_cand_1(i) * ones(rsl,1);
        U_str = [u1,u2,u3];
        MP_Inputs{1}{(j-1)*length(u3_cand_1)+i} = U_str;
    end
end

%% Make the curves somewhat following the dynamics of aircrafts. 
% 2. A long curve
% And I will move to make the next set of motion primitives.
% I adopt the simpler dynamics which five elements of a reduced state.
% X_red = [x,y,chi(course angles),gamma(flight path angle),-h]';
% Be careful heading angle is not exactly same when the speed of wind is
% comparable to the speed of air-relative velocity.
% The number of u2 is six.
u2_cand_2 = [deg2rad(30/Ts),deg2rad(-30/Ts),deg2rad(60/Ts),deg2rad(-60/Ts),deg2rad(90/Ts),deg2rad(-90/Ts)];
% The number of u3 is five.
u3_cand_2 =[deg2rad(45),deg2rad(-45),deg2rad(15),deg2rad(-15),0];
N2 = length(u1_cand) * length(u2_cand_2) * length(u3_cand_2);

MP_Inputs{2} = cell(N2,1);
MP_Inputs_cuv = zeros(N2,3);
for k = 1:length(u1_cand)
    for i = 1: length(u3_cand_2)
        for j = 1:length(u2_cand_2)
            MP_Inputs_cuv((k-1)*(length(u2_cand_2)*length(u3_cand_2))+(i-1)*length(u2_cand_2)+j,:) = [u1_cand(k),u2_cand_2(j),u3_cand_2(i)];
        end
    end
end
for i = 1:length(MP_Inputs_cuv)
    MP_Inputs{2}{i} = repmat(MP_Inputs_cuv(i,:),rsl,1);
end

% [u1,u2,u3], u1 is firstly fixed and u3 is firstly fixed.
%% 3. Spirals.
% How many circles, How far go upward?
% a half circles -> 180/Ts,-180/Ts
% How many circles -> if a retotaion is enough, 360/Ts, -360/Ts
% two rotations : 720/Ts, -720/Ts
% three rotations : 1080/Ts,-1080/Ts

% How far go upward?
% Flight path angle is 45,-45,10,-10,0
% The number of u2 is eight.
u2_cand_3 = deg2rad(360/Ts)*[0.5,-0.5,1,-1,2,-2,3,-3];
% The number of u3 is five.
u3_cand_3 =[deg2rad(45),deg2rad(-45),deg2rad(15),deg2rad(-15),0];
N3 = length(u1_cand) * length(u2_cand_3) * length(u3_cand_3);

MP_Inputs{3} = cell(N3,1);
MP_Inputs_spr = zeros(N3,3);
for k = 1:length(u1_cand)
    for i = 1: length(u3_cand_3)
        for j = 1:length(u2_cand_3)
            MP_Inputs_spr((k-1)*(length(u3_cand_3)*length(u2_cand_3))+(i-1)*length(u2_cand_3)+j,:) = [u1_cand(k),u2_cand_3(j),u3_cand_3(i)];
        end
    end
end
for i = 1:length(MP_Inputs_spr)
    MP_Inputs{3}{i} = repmat(MP_Inputs_spr(i,:),rsl,1);
end


%% 4. Splined curves.
% Connect the curves which has a relatively large curvature.
u2_cand_4 = [deg2rad(90/Ts),deg2rad(-90/Ts),deg2rad(60/Ts),deg2rad(-60/Ts)];
u2_cand_4 = u2_cand_4/2;
u2_cand_4_another = u2_cand_4;
u3_cand_4 = [0];
N4 = length(u1_cand) * length(u2_cand_4) * length(u2_cand_4_another);
MP_Inputs{4} = cell(N4,1);
for i = 1:N4
    
    MP_Inputs{4}{i} = cell(2,1);

end
int = [];
for k = 1:length(u1_cand)
    for i = 1: length(u2_cand_4_another)
        for j = 1:length(u2_cand_4)
            a = [u1_cand(k),u2_cand_4(j),u3_cand_4(1)];
            b = [u1_cand(k),u2_cand_4_another(i),u3_cand_4(1)];
            idx = (k-1)*(length(u2_cand_4_another)*length(u2_cand_4))+(i-1)*length(u2_cand_4_another)+j;
            MP_Inputs{4}{idx}{1} = repmat(a,rsl,1);
            MP_Inputs{4}{idx}{2} = repmat(b,rsl,1);
            if MP_Inputs{4}{idx}{1}(:,2:3) == MP_Inputs{4}{idx}{2}(:,2:3)
                int = [int,idx];
            end
        end
    end
end
MP_Inputs{4}(int) = [];

%celldisp(MP_Inputs)

%% 5. Visualization
if flag == 'VIS'
    figure;
    
    str_MP = MP_Inputs{1};
    cuv_MP = MP_Inputs{2};
    spr_MP = MP_Inputs{3};
    spl_MP = MP_Inputs{4};
    L = length(str_MP) + length(cuv_MP) + length(spr_MP) + length(spl_MP);
    Trajectories = cell(4,1);
    Trajectories{1} = cell(N1,1);
    Trajectories{2} = cell(N2,1);
    Trajectories{3} = cell(N3,1);
    Trajectories{4} = cell(N4,1);
    for flag = 1:length(MP_Inputs)
        for i = 1:length(MP_Inputs{flag})
            Trjt = [];
            x_old = [0,0,0,-1655]';
            for k = 1:rsl
                Trjt = [Trjt,x_old];
                if flag == 4
                    if k > rsl/2
                        x_del = MP(x_old,sml_t,MP_Inputs{flag}{i}{2}(k,:));
                    else
                        x_del = MP(x_old,sml_t,MP_Inputs{flag}{i}{1}(k,:));
                    end
                else
                    x_del = MP(x_old,sml_t,MP_Inputs{flag}{i}(k,:));
                end
                wind_effect = sml_t * [wind_inertial(1),wind_inertial(2),0,wind_inertial(3)]';
                x1 = x_old + x_del' + wind_effect;
                x_old = x1;

            end
            Trajectories{flag}{i} = Trjt;
            title("All Motion Primitives are presented",'fontsize',28)
            if flag == 1
                disp("The straight lines,Green")
                plot3(Trajectories{flag}{i}(1,:),Trajectories{flag}{i}(2,:),-Trajectories{flag}{i}(end,:),'g');
                hold on;view(3);grid on;
            elseif flag == 2
                disp("The curves,Blue")
                plot3(Trajectories{flag}{i}(1,:),Trajectories{flag}{i}(2,:),-Trajectories{flag}{i}(end,:),'b');
                        hold on;view(3);grid on;

            elseif flag == 3
                disp("The spirals,Black")
                plot3(Trajectories{flag}{i}(1,:),Trajectories{flag}{i}(2,:),-Trajectories{flag}{i}(end,:),'k');
                        hold on;view(3);grid on;
            else
                disp("The spline curves,Red")
                plot3(Trajectories{flag}{i}(1,:),Trajectories{flag}{i}(2,:),-Trajectories{flag}{i}(end,:),'r');       
                hold on;view(3);grid on;
            end
            fntsz = 28;
            xlabel("x [m]",'fontsize',fntsz);ylabel("y [m]",'fontsize',fntsz);zlabel("z [m]",'fontsize',fntsz);
            plot3(0,0,0,'markerfacecolor','red')
            

        end
    end
end

end

