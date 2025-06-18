function[result,verifyMat] = RotationMatrixFunc(angle1,angle2,angle3,rot1,rot2,rot3)
% Inputs:   angle1 = angle about first axis of rotation     {deg}
%           angle2 = angle about second axis of rotation    {deg}
%           angle3 = angle about third axis of rotation     {deg}
%           rot1 = define the which axis is rotated about first
%           rot2 = define the which axis is rotated about second
%           rot3 = define the which axis is rotated about third
%
% Outputs:  result = 3x3 rotation matrix of numerical values
%           verifyMat = debugging matrix of symbolic functions
% 
% Methodology:  create a rotation matrix by multiplying together 
%               3 single-axis rotation matrices

angle1 = deg2rad(angle1);
angle2 = deg2rad(angle2);
angle3 = deg2rad(angle3);

syms t
M(:,:,1) = [1,0,0; 0,cos(t),sin(t); 0,-sin(t),cos(t)];
M(:,:,2) = [cos(t),0,-sin(t); 0,1,0; sin(t),0,cos(t)];
M(:,:,3) = [cos(t),sin(t),0; -sin(t),cos(t),0; 0,0,1];

step = subs(M(:,:,rot2),angle2) * subs(M(:,:,rot1),angle1);
verifyMat = subs(M(:,:,rot3),angle3) * step;
result = double(verifyMat);
end

