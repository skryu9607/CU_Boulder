function MP_Input = single_MP(MP_Inputs,i)
%SINGLE_MP Summary : Output : matrix array, except cell array.
str_MP = MP_Inputs{1};
cuv_MP = MP_Inputs{2};
spr_MP = MP_Inputs{3};
spl_MP = MP_Inputs{4};
if i < length(str_MP) + 1
    MP_Input = MP_Inputs{1}{i,:};
elseif i < length(str_MP) + length(cuv_MP) + 1
    MP_Input = MP_Inputs{2}{i-length(str_MP),:};
elseif i < length(str_MP) + length(cuv_MP) + length(spr_MP) + 1
    MP_Input = MP_Inputs{3}{i-(length(str_MP)+length(cuv_MP)),:};
elseif i < length(str_MP) + length(cuv_MP) + length(spr_MP) + length(spl_MP) + 1
    MP_Input = MP_Inputs{4}{i-(length(str_MP)+length(cuv_MP)+length(spr_MP)),:};
else
    disp("The index I Exceed the maximum motion primitives")
end

end

