function [V,D] = manipulation_eig(r1, r2, t1, t2)
    J = ik_jacobian(r1, r2, t1, t2);
    M = J*J';
    [V,D] = eig(M);
end