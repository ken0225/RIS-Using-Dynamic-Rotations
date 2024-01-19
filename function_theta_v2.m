function [output_function_theta_v2]=function_theta_v2(p_mn, p_BS) 
% Author: Ke(Ken)WANG from Macao Polytechnic Institute
% Email: ke.wang@ipm.edu.mo, kewang0225@gmail.com
% Update infomation: v0.1(2020/11/19), v0.4(2022/04/30)
%
% This function aims to calculate the polar angle $\theta_T$ and $\theta_R(t)$.
% Please refer to the content below the eq(7) in GC2021 paper.
%
% License: This code is licensed under the GPLv2 license. If you in any way
% use this code for research that results in publications, please cite our
% original article.
%
% Example:
%
% p_BS = [-1,0,-1]; p_mn= [1,0,0]; [output_function_theta]=function_theta_v2(p_mn, p_BS)
% p_BS = [-1,0,-1]; p_mn= [1,0,0]; [output_function_theta]=function_theta(p_mn, p_BS)


    %Check the inputs are suitable or not
    if ismatrix(p_mn) && ismatrix(p_BS)
        
        p_mn = function_check_dim(p_mn);
        p_BS = function_check_dim(p_BS);
        
    else

        error('Please note that only the matrix is supported.') 
        
    end
    
    p_mn_2d = [p_mn(1), p_mn(3)];
    p_BS_2d = [p_BS(1), p_BS(3)];
    y=@(x)((p_BS(3)-p_mn(3))/(p_BS(1)-p_mn(1)))*(x-p_mn(1)) + p_mn(3);
    temp_y = y(0);
    
    a = norm(p_mn_2d-[0,0]); b = norm([0,temp_y]-[0,0]); c = norm([0,temp_y]-p_mn_2d);
    
    % Now we know the length of the three sides of the triangle, then we
    % calculate the angle
   output_function_theta_v2=pi/2 - acos((a^2+c^2-b^2)/(2*a*c));

end
