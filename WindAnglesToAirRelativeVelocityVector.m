function [velocity_body] = WindAnglesToAirRelativeVelocityVector(wind_angles)
%WINDANGLESTOAIRRELATIVEVELOCITYVECTOR 이 함수의 요약 설명 위치
va = wind_angles(1);
bet = wind_angles(2);
alp = wind_angles(3);
velocity_body(1) = va * cos(alp) * cos(bet);
velocity_body(2) = va * sin(bet);
velocity_body(3) = va * sin(alp) * cos(bet);

end

