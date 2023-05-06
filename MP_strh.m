function X_dot = MP_strh(va,wind_inertial)
% Steady Level , gamma = 0, deg2rad(5), deg2rad(-5)
% Course_angle = 0,deg2rad(5),deg2rad(-5)
% [P_n, P_e, h, va, course angle, flight path angle]
% Inputs = [Course angle, Flight path angle]
% Inputs = [0,0,0,deg2rad(5),0,deg2rad(-5),deg2rad(5),0,deg2rad(5),deg2rad(5)...
%     deg2rad(5),deg2rad(-5),deg2rad(-5),0,deg2rad(-5),deg2rad(5),deg2rad(-5),deg2rad(-5)];
Inputs = [0,0,deg2rad(10),0,deg2rad(-10),0,0,deg2rad(10),0,deg2rad(-10)];
X_dot = [];
for i = 1:2:length(Inputs)-1
    X_dot = [X_dot,[va * cos(Inputs(i)) * cos(Inputs(i+1)) + wind_inertial(1),...
        va * sin(Inputs(i)) * cos(Inputs(i+1)) + wind_inertial(2),...
        +va * sin(Inputs(i+1)) - wind_inertial(3)]'];
end



% % Trim_id == 1, steady level.
% MP.straight = zeros(3,4);
% MP.straight(1,:) = Trim_box(as,ap,R_t,1,0)';
% MP.straight(2,:) = Trim_box(as,ap,R_t,1,1)';
% MP.straight(3,:) = Trim_box(as,ap,R_t,1,-1)';
% % Bank TURN WITHOUT HEIGHT CHANGE.
% % R.small, R.large , L.small, L.large
% MP.turn_wo_climb = zeros(4,4);
% MP.turn_wo_climb(1,:) = Trim_box(as,ap,R_t(1),2,0)';
% MP.turn_wo_climb(2,:) = Trim_box(as,ap,R_t(2),2,0)';
% MP.turn_wo_climb(3,:) = Trim_box(as,ap,-R_t(1),2,0)';
% MP.turn_wo_climb(4,:) = Trim_box(as,ap,-R_t(2),2,0)';
% % Climb TURN
% % R.small.climb,R.small.descend, R.large.climb,R.large.descend
% MP.turn_climb = zeros(8,4);
% MP.turn_climb(1,:) = Trim_box(as,ap,R_t(1),2,1);
% MP.turn_climb(2,:) = Trim_box(as,ap,R_t(1),2,-1);
% MP.turn_climb(3,:) = Trim_box(as,ap,R_t(2),2,1);
% MP.turn_climb(4,:) = Trim_box(as,ap,R_t(2),2,-1);
% % L.small.climb,L.small.descend, L.large.climb,L.large.descend
% MP.turn_climb(5,:) = Trim_box(as,ap,-R_t(1),2,1);
% MP.turn_climb(6,:) = Trim_box(as,ap,-R_t(1),2,-1);
% MP.turn_climb(7,:) = Trim_box(as,ap,-R_t(2),2,1);
% MP.turn_climb(8,:) = Trim_box(as,ap,-R_t(2),2,-1);

end

