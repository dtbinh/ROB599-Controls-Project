function u_k = get_u(A, B, x_k, k, u_ref, khorizon)

    zsize = (khorizon+1)*6+khorizon*2; 
    xsize = (khorizon+1)*6; 

    Q = 1000*eye(xsize); 
    R = zeros(zsize-xsize); 
    H = blkdiag(Q, R);
    f = zeros(zsize, 1); 
    
    [Aeq, beq] = eq_cons(A,B,x_k,u_ref(k,:),khorizon); 
    Aineq=[];
    bineq = [];
    z = quadprog(H, f, Aineq, bineq, Aeq, beq); 
    u_k = z([xsize+1 xsize+2],1);
    
end
