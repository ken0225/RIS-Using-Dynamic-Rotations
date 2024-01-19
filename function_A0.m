function [A0]=function_A0(p1,p2)  % p2 denotes Receiver and p1 denotes Transmitter
% Author: Ke(Ken)WANG from Macao Polytechnic Institute
% Email: ke.wang@ipm.edu.mo, kewang0225@gmail.com
% Update infomation: v0.1(2020/11/19), v0.2(2021/08/26)
%
% This function aims to calculate the vehicle coordinate increment.
% Note that we suppose the vehicle only moves along the positive x-axis direction
%
% License: This code is licensed under the GPLv2 license. If you in any way
% use this code for research that results in publications, please cite our
% original article.
%
% Example:
%
% [A0]=function_amplitude_gains_A0([1,2,3], [4,5,6])
%
% A0 =
%
%     0.0019


global c0 fc;

lambda_c = c0/fc;

%The gain of transmitter, from the direction of receiver
G_T_to_R_direction = function_antenna_gain_TR(p1,p2);

%The gain of receiver, from the direction of transmitter
G_R_to_T_direction = function_antenna_gain_TR(p2,p1);

A0 = lambda_c/(4*pi) * sqrt(G_T_to_R_direction.*G_R_to_T_direction)./vecnorm((p2-p1).'); %eq(3) in GC2021 paper

end
