function [X_del] = MP(X_old,Ts,MP_Input)
% Full state equations to describe the next step of states
% The meaning of using MP, to extend the branch. 
% X = [x,y,chi,gamma,va,h]
% Splined curves
if isa(MP_Input,"cell")
    a = MP_Input{1};
    b = MP_Input{2};
    va = a(1);
    u2 = a(2);
    u3 = a(3);
    if u2 == 0
        X_del(1) = va*cos(X_old(3))*cos(u3)*Ts;
        X_del(2) = va*sin(X_old(3))*cos(u3)*Ts;
        X_del(3) = 0;
        X_del(4) = -va*sin(u3)*Ts;
    else
    %     X_del(1) = va*cos(X(3))*cos(u3)*Ts;
    %     X_del(2) = va*sin(X(3))*cos(u3)*Ts;
    %     X_del(3) = u2*Ts;
    %     X_del(4) = 0;
    %     X_del(5) = -va*sin(u3)*Ts;
        X_del(1) = va*cos(u3)/u2*(sin(u2*Ts+X_old(3))-sin(X_old(3)));
        X_del(2) = va*cos(u3)/u2*(cos(X_old(3))-cos(u2*Ts+X_old(3)));
        X_del(3) = u2*Ts;
        X_del(4) = -va*sin(u3)*Ts;
    end
    va = b(1);
    u2 = b(2);
    u3 = b(3);
    if u2 == 0
        X_del(1) = va*cos(X_old(3))*cos(u3)*Ts;
        X_del(2) = va*sin(X_old(3))*cos(u3)*Ts;
        X_del(3) = 0;
        X_del(4) =  - va*sin(u3)*Ts;
    else
    %     X_del(1) = va*cos(X(3))*cos(u3)*Ts;
    %     X_del(2) = va*sin(X(3))*cos(u3)*Ts;
    %     X_del(3) = u2*Ts;
    %     X_del(4) = 0;
    %     X_del(5) = -va*sin(u3)*Ts;
        X_del(1) = va*cos(u3)/u2*(sin(u2*Ts+X_old(3))-sin(X_old(3)));
        X_del(2) = va*cos(u3)/u2*(cos(X_old(3))-cos(u2*Ts+X_old(3)));
        X_del(3) = u2*Ts;
        X_del(4) = -va*sin(u3)*Ts;
    end
% Straight, Curves, Spirals
else
    va = MP_Input(1);
    u2 = MP_Input(2);
    u3 = MP_Input(3);
    if u2 == 0
        X_del(1) = va*cos(X_old(3))*cos(u3)*Ts;
        X_del(2) = va*sin(X_old(3))*cos(u3)*Ts;
        X_del(3) = 0;
        X_del(4) = -va*sin(u3)*Ts;
    else
    %     X_del(1) = va*cos(X(3))*cos(u3)*Ts;
    %     X_del(2) = va*sin(X(3))*cos(u3)*Ts;
    %     X_del(3) = u2*Ts;
    %     X_del(4) = 0;
    %     X_del(5) = -va*sin(u3)*Ts;
        X_del(1) = va*cos(u3)/u2*(sin(u2*Ts+X_old(3))-sin(X_old(3)));
        X_del(2) = va*cos(u3)/u2*(cos(X_old(3))-cos(u2*Ts+X_old(3)));
        X_del(3) = u2*Ts;
        X_del(4) = -va*sin(u3)*Ts;
    end
end

% % X = [Pn,Pe,ki,gam,h]
% global va
% u2 = MP_Input(1);
% u3 = MP_Input(2);
% % Make the repeated terms
% add = (u2+u3)*Ts + X(3) + X(4);
% min = (u2-u3)*Ts + X(3) - X(4);
% % Compute the Pn
% if u2 == 0 && u3 == 0
%     X_dot(1) = va * cos(X(3)) * cos(X(4)) * Ts;
%     %X(1) = X(1) + 0(X(1));
% elseif u2 - u3 == 0
%     X_dot(1) = va/2*cos(X(3)-X(4))*Ts + va/(2*(u2+u3))*(sin(add)-sin(X(3)+X(4)));
% elseif u2 + u3 == 0
%     X_dot(1) = va/2*cos(X(3)+X(4))*Ts + va/(2*(u2-u3))*(sin(min)-sin(X(3)-X(4)));
% else
%     X_dot(1) = va/(u2-u3)*cos(min)*sin((u2-u3)/2*Ts) + va/(u2+u3)*cos(add)*sin((u2+u3)/2*Ts);
% end
% % Compute the Pe
% if u2 == 0 && u3 == 0
%     X_dot(2) = va * sin(X(3)) * cos(X(4)) * Ts;
% elseif u2 - u3 == 0
%     X_dot(2) = va/2*sin(X(3)-X(4))*Ts + va/(2*(u2+u3))*(-cos(add)+cos(X(3)+X(4)));
% elseif u2 + u3 == 0
%     X_dot(2) = va/2*sin(X(3)+X(4))*Ts + va/(2*(u2-u3))*(-cos(min)+cos(X(3)-X(4)));
% else
%     X_dot(2) = va/(u2-u3)*cos(min)*sin((u2-u3)/2*Ts) + va/(u2+u3)*cos(add)*sin((u2+u3)/2*Ts);
% end
% % Compute the h
% if u3 == 0
%     X_dot(5) = va*sin(X(4))*Ts;
% else
%     X_dot(5) = va/u3*(cos(X(4))-cos(u3*Ts+X(4)));
% end
% % Course angle ki
% X_dot(3) = u2 * Ts;
% X_dot(4) = u3 * Ts;

        

end

