function u = do_MPC(G,F,A_ineq,B_ineq,p)
    % Inputs
    % G (augmented cost)                        : p(N) x p(N)
    % F (augmented cost)                        : p(N) x 1
    % augmented cost                            : 1/2 * X^T * G * X + X^T * F * x0
    % A_ineq (augmented ineq constraint A)      : (p+m)(N) x (p+m)(N)
    % b_ineq (augmented ineq constraint b)      : 2(p+m)(N) x 1
    % b_ineq_x0 (augmented ineq constraint cx)  : 2(p+m)(N) x 1
    % augmented ineq constraint                 : A_ineq * X <= b_ineq + b_ineqx0 * x0
    % x0 (intial condititon for each horizon)   : n x 1
    % u_min is min the control bound            : scalar
    % u_max is max the control bound            : scalar
    % y_min is min the output bound             : scalar
    % y_max is min the output bound             : scalar
    % Q_i and R_i is assumed to be constant for i = 1 to N
    % Outputs
    % u (MPC output)                            : p x 1
    
    options = optimoptions('quadprog','Display','off');
    U = quadprog(G,F,A_ineq,B_ineq,[],[],[],[],[],options);
    u = U(1:p);
    
end