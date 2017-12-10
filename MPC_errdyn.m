%Reference input

U1 = [-0.005*ones(900,1) 1600*ones(900,1)];
U2 = [-0.19*ones(665,1) 0*ones(665,1)];
U3 = [0*ones(800,1) 0*ones(800,1)];
U4 = [0.06*ones(375,1) 1000*ones(375,1)];
U5 = [-0.2*ones(500,1) 600*ones(500,1)];
U6 = [0.1*ones(425,1) 0*ones(425,1)];
U7 = [0*ones(300,1) 800*ones(300,1)];
U8 = [-0.045*ones(825,1) 300*ones(825,1)];
U9 = [-0.001*ones(375,1) 0*ones(375,1)];
U10 = [0*ones(25,1) -9000*ones(25,1)];
U11 = [0.01*ones(300,1) 0*ones(300,1)];
U12 = [0.15*ones(400,1) 0*ones(400,1)];
U13 = [-0.010*ones(700,1) 200*ones(700,1)];
U14 = [-0.12*ones(600,1) 300*ones(600,1)];
U15 = [0.1*ones(400,1) 0*ones(400,1)];
U16 = [0.0012*ones(650,1) 1750*ones(650,1)];
U17 = [0.08*ones(250,1) 0*ones(250,1)];
U18 = [0.18*ones(135,1) 0*ones(135,1)];
U19 = [0*ones(800,1) 4000*ones(800,1)];

u_ref = [U1;U2;U3;U4;U5;U6;U7;U8;U9;U10;U11;U12;U13;U14;U15;U16;U17;U18;U19];


%Initialization
Aineq=[];
Bineq=[];
x = [287 5 -176 0 2 0]';
u = [-0.005 1600];
dt = 0.01;
t=0:dt:94.25;

%MPC at every time step k
for k = 1:length(t)-1
    if length(t)-k>10
        khorizon = 5 ; 
    else
        khorizon = length(t)-k; 
    end
    
    %Evaluate Al and Bl at current state
[Al,Bl]=linearized_mats(x(:,k)',u(k,:));

%Discretize (Euler)
A = eye(size(Al))+dt*Al;
B = dt*Bl;

    u_k = get_u(A, B, x(:, k), k, u_ref, khorizon) ; %obtaining the error in input
    u(k+1,:) = u_k'+u_ref(k,:); %augmenting with nominal input
    
    %Simulating using the total input
    if k==1
[L]=forwardIntegrateControlInput([-0.005 1600;u(k,:)],x(:,k)');
else 
    [L]=forwardIntegrateControlInput([u(k:k+1,:)],x(:,k)');
    end

x=[x L(end,:)']; %updating states
end



function u_k = get_u(A, B, x_k, k, u_ref, khorizon)
    zsize = (khorizon+1)*6+khorizon*2 ; 
    xsize = (khorizon+1)*6 ; 

    Q = 1000*eye(xsize) ; 
    R = zeros(zsize-xsize) ; 
    H = blkdiag(Q, R) ;
    f = zeros(zsize, 1) ; 
    
    [Aeq, beq] = eq_cons(A, B, x_k,u_ref(k,:),khorizon) ; 
    Aineq=[];
    bineq = [];
    z = quadprog(H, f, Aineq, bineq, Aeq, beq) ; 
    u_k = z([xsize+1 xsize+2],1) ; 
end


function [Aeq, beq] = eq_cons(A, B, x_k, u_k, horizon)

n = length(x_k);                    %Number of States
m = length(u_k);                    %Number of Inputs

xsize = (horizon+1)*n;              %Number of State Decision Variables
zsize = (horizon+1)*n+horizon*m;    %Total Number of Decision Variables

Aeq = zeros(xsize, zsize);          %Allocate Aeq
Aeq(1:n, 1:n) = eye(n);             %Initial Condition LHS

beq = zeros(xsize, 1);              %Allocate beq
beq(1:n) = x_k;                     %Initial Condition RHS

j = xsize+1;

for i = n+1:n:xsize
    
    Aeq(i:i+n-1, i:i+n-1) = -eye(n);    %x(k+1) term
    Aeq(i:i+n-1, i-n:i-1) = A;          %A*x(k) term
    Aeq(i:i+n-1, j:j+m-1) = B;          %B*u(k) term
    
    j = j+m;
    
end

end