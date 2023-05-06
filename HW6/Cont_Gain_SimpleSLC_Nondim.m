function [control_gains,linear_terms] = Cont_Gain_SimpleSLC_Nondim(ap,trd0,trv0)
%% Simple Succesive Loop closure
% ap : aircraft_parameters
% trd0 : trim_definition
% trv0 : trim_variables
% Using the linear design models in Lec 5 and strategies described in Lec 7
% and 8, this function calculates the gains for the 
% "Simple Successive Loop Closure" autopilot. 

% Although the functions takes as inputs the ap file, various aspects of
% the design are specific to a given a/c, thus this function is used only
% to design gains for the Twistor. 


end

