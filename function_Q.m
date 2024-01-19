function [Q]=function_Q(A_0, A_star, A_mn_vector)

Q = (A_0)^2 + A_star + sum(A_mn_vector.^2) + 2*A_0*sum(A_mn_vector); 

end