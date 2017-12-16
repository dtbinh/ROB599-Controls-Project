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

u_ref = [U1; U2; U3; U4; U5; U6; U7; U8; U9; U10;
     U11;U12;U13;U14;U15;U16;U17;U18;U19;U20;
     U21;U22;U23;U24;U25;U26;U27;U28;U29     ];

%Initialization
Aineq=[];
Bineq=[];
x = [287 5 -176 0 2 0]';
u = [-0.02 5000];
dt = 0.1;
t=0:dt:10;%117.5;


%MPC at every time step k
for k = 1:length(t)-1
    
    if length(t)-k>5
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
        [L]=forwardIntegrateControlInput([-0.02 5000;u(k,:)],x(:,k)');
    else 
        [L]=forwardIntegrateControlInput([u(k:k+1,:)],x(:,k)');
    end

    x=[x L(end,:)']; %updating states

end

plot(bl_x,bl_y,br_x,br_y,cline_x,cline_y,'--',x(1,:),x(3,:),'k')