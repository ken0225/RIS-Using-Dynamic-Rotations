function [output_function_theta]=function_theta(p1, p2) % p2 denotes Receiver, p1 denotes Transmitter, and theta is the polar angle

% Author: Ke(Ken)WANG from Macao Polytechnic Institute
% Email: ke.wang@ipm.edu.mo, kewang0225@gmail.com
% Update infomation: v0.1(2020/11/19), v0.2(2021/08/26), v0.3(2021/09/04)
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
% p1 = [1,0,1]; p2 = [0,0,0]; [output_function_theta]=function_theta(p1, p2)
% 
% output_function_theta =
% 
%     0.7854


    %Check the inputs are suitable or not
    if ismatrix(p1) && ismatrix(p2)
        
        p1 = function_check_dim(p1);
        p2 = function_check_dim(p2);
        
    else

        error('Only matrix supported! ') 
        
    end
    
    %Please refer to the content below the eq(7) in GC2021 paper.
    %Note that "(p2(:, end) - p1(:, end)" is the z-axis coordinate 
    output_function_theta = acos(abs((p2(:, end)-p1(:, end))) ./ vecnorm((p2-p1).').');
   
    % Convert the column vector to row vector
    output_function_theta = output_function_theta.'; 
    
end
