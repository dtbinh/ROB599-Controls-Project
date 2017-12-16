clear all
close all
clc
%%
load('TestTrack.mat')

bl_x = TestTrack.bl(1,:);
bl_y = TestTrack.bl(2,:);

br_x = TestTrack.br(1,:);
br_y = TestTrack.br(2,:);

cline_x = TestTrack.cline(1,:);
cline_y = TestTrack.cline(2,:);

theta = TestTrack.theta(1,:);
%% Reference input

U1 = [-0.02*ones(143,1) 5000*ones(143,1)];
U2 = [0*ones(800,1) 200*ones(800,1)];
U3 = [-0.04*ones(200,1) 0*ones(200,1)];
U4 = [-0.022*ones(400,1) 0*ones(400,1)];
U5 = [-0.037*ones(260,1) 0*ones(260,1)];
U6 = [0*ones(1100,1) 200*ones(1100,1)];
U7 = [0.055*ones(260,1) 0*ones(260,1)];
U8 = [0*ones(300,1) 200*ones(300,1)];
U9 = [-0.07*ones(325,1) 0*ones(325,1)];
U10 = [0*ones(200,1) 200*ones(200,1)];
U11 = [0.07*ones(400,1) 0*ones(400,1)];
U12 = [0*ones(280,1) 200*ones(280,1)];
U13 = [-0.032*ones(475,1) 0*ones(475,1)];
U14 = [-0.015*ones(400,1) 200*ones(400,1)];
U15 = [-0.045*ones(398,1) 200*ones(398,1)];
U16 = [0*ones(825,1) -250*ones(825,1)];
U17 = [0.135*ones(270,1) 0*ones(270,1)];
U18 = [0.0035*ones(400,1) 200*ones(400,1)];
U19 = [0.025*ones(350,1) 200*ones(350,1)];
U20 = [-0.01*ones(250,1) 200*ones(250,1)];
U21 = [-0.054*ones(600,1) 0*ones(600,1)];
U22 = [-0.05*ones(425,1) 0*ones(425,1)];
U23 = [0.04*ones(120,1) 0*ones(120,1)];
U24 = [0.125*ones(305,1) 0*ones(305,1)];
U25 = [0*ones(80,1) 5000*ones(80,1)];
U26 = [0*ones(1120,1) 0*ones(1120,1)];
U27 = [0.075*ones(223,1) 0*ones(223,1)];
U28 = [0*ones(700,1) 5000*ones(700,1)];
U29 = [0.1*ones(150,1) 5000*ones(150,1)];

u_ol = [U1; U2; U3; U4; U5; U6; U7; U8; U9; U10;
     U11;U12;U13;U14;U15;U16;U17;U18;U19;U20;
     U21;U22;U23;U24;U25;U26;U27;U28;U29     ];

%% Run open loop to get equilibrium trajectory
x0 = [287.1 5 -176 0 2 0]';
x_ol = forwardIntegrateControlInput(u_ol,x0);
t_sim=0:0.01:(size(u_ol,1)-1)*0.01;

%% Obstacle generation and collision detection
nobs=40;%no of obstacles
Xobs = generateRandomObstacles(nobs,TestTrack);

h1=figure;
hold on
plot(bl_x,bl_y,br_x,br_y,'b');%cline_x,cline_y,'--',
for pp=1:nobs
    plot(Xobs{pp}(1:2,1),Xobs{pp}(1:2,2),'-r')
    plot(Xobs{pp}(2:3,1),Xobs{pp}(2:3,2),'-g')
    plot(Xobs{pp}(3:4,1),Xobs{pp}(3:4,2),'-b')
    plot(Xobs{pp}([4 1],1),Xobs{pp}([4 1],2),'-m')
    obs_x_mat(:,pp)=Xobs{pp}(1:4,1);
    obs_y_mat(:,pp)=Xobs{pp}(1:4,2);
    obs_c_mat(:,pp)=[mean(Xobs{pp}(1:4,1));mean(Xobs{pp}(1:4,2))];
end

t_sim=0:0.01:(size(u_ol,1)-1)*0.01;
%%

%Initialization
detect_dist=sqrt(2)*5;%Detection distance of obstacle in m
dt = 0.1;
t=0:dt:100*dt;%117.5;

x_ref=interp1(t_sim,x_ol,t);
u_ref=interp1(t_sim,u_ol,t);

figure(h1);
hold on
plot(x_ref(:,1),x_ref(:,3),'-.m');

x = [289 5 -175 0 2 0]';%[287 5 -176 0 2 0]';
e = x-x_ref(1,:)';
u = u_ref(1,:);%[-0.02 5000];

%Normal parameters
n = 6;                    %Number of States
m = 2;                    %Number of Inputs
horizon=5;
zsize = (horizon+1)*n+horizon*m; 
xsize = (horizon+1)*n; 
Q = diag(repmat((ones(1,6)./[100 10 100 10 0.1 0.01]).^2,1,horizon+1));%eye(xsize);%
R = zeros(zsize-xsize);%diag(repmat([1 1]./[0.01 1000],1,horizon));%
H = blkdiag(Q, R);
f = zeros(zsize, 1); 
Ndec=n * (horizon+1) + m *horizon ;
options=optimoptions(@fmincon,'Algorithm','sqp');%,'Display','Iter');%,'CheckGradients',false,'GradObj','on','ConstraintTolerance',1e-6,'MaxIter',10000,'MaxFunctionEvaluations',50000,'Display','Iter');%,[repmat([100;5;100;5;1;0.1],(PredHorizon+1),1) ; repmat([0.1;1000],PredHorizon,1)]'TypicalX',sqrt(1./diag(Q))

%MPC at every time step k
for k = 1:length(t)-1
    disp(['At ' num2str(t(k)) ' s:'])
    
    if ~(length(t)-k>5)
    horizon = length(t)-k; 
    zsize = (horizon+1)*n+horizon*m; 
    xsize = (horizon+1)*n; 

    Q = 1000*eye(xsize); 
    R = zeros(zsize-xsize); 
    H = blkdiag(Q, R);
    f = zeros(zsize, 1); 
    Ndec=n * (horizon+1) + m *horizon ;
    end
 
    %Evaluate Al and Bl at current state
    [Al,Bl]=linearized_mats(x(:,k)',u(k,:));

    %Discretize (Euler)
    A = eye(size(Al))+dt*Al;
    B = dt*Bl;

%% Generate equality constraints

Aeq = zeros(xsize, zsize);          %Allocate Aeq
Aeq(1:n, 1:n) = eye(n);             %Initial Condition LHS
beq = zeros(xsize, 1);              %Allocate beq
beq(1:n) = e(:,end);                     %Initial Condition RHS

j = xsize+1;

for i = n+1:n:xsize
    
    Aeq(i:i+n-1, i:i+n-1) = -eye(n);    %x(k+1) term
    Aeq(i:i+n-1, i-n:i-1) = A;          %A*x(k) term
    Aeq(i:i+n-1, j:j+m-1) = B;          %B*u(k) term
    
    j = j+m;
    
end
   
%     z = quadprog(H, f, Aineq, bineq, Aeq, beq); 
    fun=@(var)(0.5*var'*H*var+f'*var);
    nlcons=@(var)track_nlcons_err(var,reshape(x_ref(k:k+horizon,:)',n*(horizon+1),1),TestTrack,Ndec,horizon,n,m);%Nonlinear track constraints
    %Simulate open loop for initial guess
    z0=zeros(Ndec,1);
    e_curr=x(:,k)-x_ref(k,:)';
    z0([1:n])=e_curr;
    for ll=2:horizon+1
    e_curr=A*e_curr;%+B*u(k,:)';
    z0((ll-1)*n+[1:n])=e_curr;
    end
    %% Detect how many obstacles are close at initial time
    
  [d2_det,ind_det]=find(((x(1,k)- obs_c_mat(1,:)).^2+(x(3,k)- obs_c_mat(2,:)).^2)<detect_dist^2);%Index of obstacles which are close enough
  n_det=length(ind_det); %No of closest obstacles detected
  if n_det>0
      figure(h1)
      plot(x(1,k),x(3,k),'sr')
      keyboard
 
  for pp=1:n_det
         %Perpendicular distances from left side of obstacle to left side of track
          pdist1=perp_dist(obs_x_mat(1,ind_det(pp)),obs_y_mat(1,ind_det(pp)),TestTrack.bl);
          pdist4=perp_dist(obs_x_mat(4,ind_det(pp)),obs_y_mat(4,ind_det(pp)),TestTrack.bl);
          pdistl=min(pdist1,pdist4);
          %Perpendicular distances from right side of obstacle to right side of track
          pdist2=perp_dist(obs_x_mat(2,ind_det(pp)),obs_y_mat(2,ind_det(pp)),TestTrack.br);
          pdist3=perp_dist(obs_x_mat(3,ind_det(pp)),obs_y_mat(3,ind_det(pp)),TestTrack.br);
          pdistr=min(pdist2,pdist3);
      %Which side of obstacle is closest to car
      [mind2,indmin]=min((x(1,k)- obs_x_mat(1:2,ind_det(pp))).^2+(x(3,k)- obs_y_mat(1:2,ind_det(pp))).^2)
      
      if (indmin==1 && pdistl>lim_pdist) %left side is closer and there is space to go on that side
          %See if there is space on left side between track and obstacle
          min(pdist1,pdist2)>
      elseif
          
      else
          
      end
   end
%% Inequality constraints

Aineq = zeros( 2*m*horizon, Ndec);
bineq = zeros( 2*m*horizon, 1 );
Aineq(1:m*horizon,n * (horizon+1)+[1:m*horizon])=eye(m*horizon);
bineq([1:2:m*horizon],1)=0.5-u_ref(k:k+horizon-1,1);
bineq([2:2:m*horizon],1)=5000-u_ref(k:k+horizon-1,2);

Aineq(m*horizon+[1:m*horizon],n * (horizon+1)+[1:m*horizon])=-eye(m*horizon);
bineq(m*horizon+[1:2:m*horizon],1)=0.5+u_ref(k:k+horizon-1,1);
bineq(m*horizon+[2:2:m*horizon],1)=10000+u_ref(k:k+horizon-1,2);

    
    %% Minimize
%     z0(n*(horizon+1)+[1:m*horizon])=repmat(u(k,:)',1,horizon);
    [z,fcostval,exitflag,output] = fmincon(fun,z0,Aineq,bineq,Aeq,beq,[],[],nlcons,options);
    u_k = z([xsize+1 xsize+2],1);

    u(k+1,:) = u_k'+u_ref(k,:); %augmenting with nominal input
    
%     Simulating using the total input
      [L]=forwardIntegrate_vardt([u(k:k+1,:)],x(:,k)',dt);
  
    x=[x L(2,:)']; %updating states
    e= [e L(2,:)'-x_ref(k,:)'];
end
%% Re-interpolate final input vector and re-calculate trajectory with new input
u_final=interp1(t,u,t_sim);
x_final = forwardIntegrateControlInput(u_final,x(:,1));
figure(h1);
plot(x_final(:,1),x_final(:,3),'--k');

figure;
plot(t,e)
legend('e_x','e_u','e_y','e_v','e_{\psi}','e_{r}')

