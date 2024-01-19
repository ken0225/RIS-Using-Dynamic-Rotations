function [output_gain_IRS]=function_gain_IRS_v2(p1, p2, dx, dy, isotropic) % p_mn=p1, p_BS=p2
% Author: Ke(Ken)WANG from Macao Polytechnic Institute
% Email: ke.wang@ipm.edu.mo, kewang0225@gmail.com
% Update infomation: v0.1(2020/11/19), v0.4(2022/04/30)
%
% This function aims to calculate the gain of an IRS (isotropic or non-isotropic)
%
% License: This code is licensed under the GPLv2 license. If you in any way
% use this code for research that results in publications, please cite our
% original article.
%
% Example:
%
% p1 = [-1,0,0;1,0,0];p2 = [3,0,3]; dx=0.1;dy=0.2;isotropic = 0; [output_gain_IRS]=function_gain_IRS_v2(p1, p2, dx, dy, isotropic)
% output_gain_IRS =
%
%    9.6643   13.4020

global c0 fc;

lambda_c = c0/fc;

for aa = 1:size(p1,1)
    temp_theta_p1_p2 = function_theta_v2(p1(aa,:), p2);
    theta_p1_p2(aa) = temp_theta_p1_p2;
end

%Note that if an IRS is isotropic, then it can be seen as a perfect
%scatterer, which means the reflect gain is maximal, i.e., 1
if isotropic == 1
    
    output_gain_IRS = ones(size(theta_p1_p2, 1), size(theta_p1_p2, 2));
    
elseif isotropic == 0
    
    %But in practical, we have to consider the position of IRS and BS(vehicle) 
    %Please refer to the content below the eq(7) in GC2021 paper
    A = 4*pi*dx*dy*(1/lambda_c)^2;
    
    cos_anisotropic = cos(theta_p1_p2);
    
    %If the cos_polar<0, we just omit it due to the assumption that our IRS only reflect signal. It cannot be penetrated 
    cos_anisotropic(cos_anisotropic<0) = 0;
    
    %Basically the gain of IRS is always changing since the vehicle is moving
    output_gain_IRS = A*cos_anisotropic;
    
else
    
    error('Please choose if it is isotropic or not.')
    
end

end