function [A_star]=function_Astar(A_mn_vector)
%A_mn_vector是包含一秒内mn个元素的gain的行向量
%此函数计算一个向量中每个元素和其他元素的乘积的总和的和，比如a = [1,2,3]，那么此函数就计算1*(sum(a)-1) + 2*(sum(a)-2) + 3*(sum(a)-3)

temp_A_star_2 = [];

for k = 1 : size(A_mn_vector,2)

temp_A_star_1 = A_mn_vector(k) * (sum(A_mn_vector)-A_mn_vector(k));

temp_A_star_2 = [temp_A_star_2; temp_A_star_1 ];

end

A_star = sum(temp_A_star_2);

end