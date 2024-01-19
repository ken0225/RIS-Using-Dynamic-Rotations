function [output_function_rotate_IRS]=function_rotate_IRS(rotate_angle)
% author: Ke(Ken)WANG from Macao Polytechnic Institute
% email: p1909883@ipm.edu.mo
% update infomation: v0.1(2020/11/19)
% 
% This function could rotate the 3D IRS coordinate counterclockwise around
% the X axis rotate_angle degree.
%
% The INPUT is the angle of rotation and the OUTPUT is a 3x3 matrix.
%
% Example: Now we have an IRS that located in IRS = [0, 1.5, 3], then we want to
% rotate it pi/4 degree counterclockwise around the X axis. The code is as follows:
%
%>> IRS = [10, 10, 0]; IRS * function_rotate_IRS(pi/2)
% 
% ans =
% 
%     0.0000   10.0000  -10.0000
    
    % 绕y轴逆时针旋转(rotate_angle>0时)
    output_function_rotate_IRS = [cos(rotate_angle), 0, sin(rotate_angle); 0, 1, 0; -sin(rotate_angle), 0, cos(rotate_angle)]';

end


% IRS = [10, 10, 0]; IRS * function_rotate_IRS(-pi/4)
% 
% ans =
% 
%     7.0711   10.0000   7.0711








