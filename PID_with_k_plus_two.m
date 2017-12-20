clear all
close all
clc
ticid=tic;

%%
load('TestTrack.mat')

bl_x = TestTrack.bl(1,:);
bl_y = TestTrack.bl(2,:);

br_x = TestTrack.br(1,:);
br_y = TestTrack.br(2,:);

cline_x = TestTrack.cline(1,:);
cline_y = TestTrack.cline(2,:);

theta = TestTrack.theta(1,:);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DELETE THIS SECTION WHEN MODIFYING FOR SUBMISSION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%Obstacle generation

rng(42)
nobs=10;%no of obstacles
Xobs = generateRandomObstacles(nobs,TestTrack);

%% Run open loop to get equilibrium trajectory
t_sim=0:0.01:400;
drift_control_step=2500;%Simulate ode45 open loop at these many time steps from start
track_fine=interp1([1:length(TestTrack.cline)],TestTrack.cline',[1:(length(TestTrack.cline)/length(t_sim)):length(TestTrack.cline)]);

%% Generate reference racing-line
ref_line=track_fine';
bend_start=10;
bend_end=10;
obs_pad=1;
h1=figure;
hold on
plot(bl_x,bl_y,br_x,br_y,'b');
for pp=1:nobs
    obs_x_mat(:,pp)=Xobs{pp}(1:4,1);
    obs_y_mat(:,pp)=Xobs{pp}(1:4,2);
    obs_c_mat(:,pp)=[mean(Xobs{pp}(1:4,1));mean(Xobs{pp}(1:4,2))];
    obs_fm_mat(:,pp)=[mean(Xobs{pp}(1:2,1));mean(Xobs{pp}(1:2,2))];
    obs_bm_mat(:,pp)=[mean(Xobs{pp}(3:4,1));mean(Xobs{pp}(3:4,2))];
    th_obs(pp)=atan2(obs_bm_mat(2,pp)- obs_fm_mat(2,pp),obs_bm_mat(1,pp)- obs_fm_mat(1,pp));
    R_obs=[cos(th_obs(pp)) sin(th_obs(pp));-sin(th_obs(pp)) cos(th_obs(pp))];%Rotation matrix to get from world CS to obstacle CS
    
    % Closest point on centerline to front and back of obstacle
    [~,indt2]=min((obs_fm_mat(1,pp)-ref_line(1,:)).^2+(obs_fm_mat(2,pp)-ref_line(2,:)).^2);
    [~,indt3]=min((obs_bm_mat(1,pp)-ref_line(1,:)).^2+(obs_bm_mat(2,pp)-ref_line(2,:)).^2);
 
    
    %Which side of obstacle is closest to reference line
    [mind2,indmin]=min((ref_line(1,indt2)- obs_x_mat(1:2,pp)).^2+(ref_line(2,indt2)- obs_y_mat(1:2,pp)).^2);
        
    if (indmin==1 ) %left side is closer 
          xl_t2=R_obs*[obs_x_mat(1,pp) ; obs_y_mat(1,pp)]+[0; obs_pad];
          xl_t3=R_obs*[obs_x_mat(4,pp) ; obs_y_mat(4,pp)]+[0 ;obs_pad];
    elseif (indmin==2) %right side is closer 
          xl_t2=R_obs*[obs_x_mat(2,pp) ; obs_y_mat(2,pp)]+[0 ;-obs_pad];
          xl_t3=R_obs*[obs_x_mat(3,pp) ; obs_y_mat(3,pp)]+[0 ;-obs_pad];
    end
         obs_width(pp)=xl_t3(1,1)-xl_t2(1,1);
       
    % Closest point on centerline to front and back of obstacle at given
    % distances
    obs_f_l=R_obs*obs_fm_mat(:,pp);
    obs_b_l=R_obs*obs_bm_mat(:,pp);
    ref_line_l=R_obs*ref_line;
    
    [~,indt1]=min((obs_f_l(1)-10-ref_line_l(1,:)).^2+(obs_f_l(2)-ref_line_l(2,:)).^2);
    [~,indt4]=min((obs_b_l(1)+10-ref_line_l(1,:)).^2+(obs_b_l(2)-ref_line_l(2,:)).^2);
    
    xl_t1=ref_line_l(:,indt1);
    xl_t4=ref_line_l(:,indt4);
    
    ref_line_rep_l=zeros(2,indt4-indt1,1);    
    ref_line_rep_l(:,[indt1:indt2]-indt1+1)=transpose(interp1([indt1 indt2]-indt1+1,[xl_t1 xl_t2]',[indt1:indt2]-indt1+1,'linear'));
    ref_line_rep_l(:,[indt2:indt3]-indt1+1)=transpose(interp1([indt2 indt3]-indt1+1,[xl_t2 xl_t3]',[indt2:indt3]-indt1+1,'linear'));
    ref_line_rep_l(:,[indt3:indt4]-indt1+1)=transpose(interp1([indt3 indt4]-indt1+1,[xl_t3 xl_t4]',[indt3:indt4]-indt1+1,'linear'));

    pts=[xl_t1 xl_t2 xl_t3 xl_t4];
    
    ref_line(:,[indt1:indt4])=R_obs'*ref_line_rep_l;

    plot(Xobs{pp}(1:2,1),Xobs{pp}(1:2,2),'-r')
    plot(Xobs{pp}(2:3,1),Xobs{pp}(2:3,2),'-g')
    plot(Xobs{pp}(3:4,1),Xobs{pp}(3:4,2),'-b')
    plot(Xobs{pp}([4 1],1),Xobs{pp}([4 1],2),'-m')
    plot(obs_fm_mat(1,pp),obs_fm_mat(2,pp),'v')
    plot(obs_bm_mat(1,pp),obs_bm_mat(2,pp),'v')
    
end

figure(h1)
plot(ref_line(1,:),ref_line(2,:),'--k')

%% PID control
t_comp_mins=15; %No. of simulation minutes allowed for competition

exit_flag=0;
k=1;
steps = 2;
dt=0.01;
x(1,:) = [287 5 -176 0 2 0];

cline_x = ref_line(1,:);
cline_y = ref_line(2,:);

for j = 1:length(ref_line)-1
theta(j) = atan2(ref_line(2,j+1)-ref_line(2,j),ref_line(1,j+1)-ref_line(1,j));
end

while (exit_flag==0) 
    
    % Exit if track completed or simtime limit reached 
    if (toc(ticid)>=0.95*t_comp_mins*60)
        exit_flag=1;
    elseif k>length(t_sim)-steps
        exit_flag=1;
    end
         
    %Stay as close to the desired max speed as possible
    if x(k,2)>10
        F_x = 0;
    else
        F_x = 200;
    end
    
    %Determine nearest point of reference trajectory
    [ind, d] = knnsearch([cline_x; cline_y]',[x(k,1) x(k,3)]);
    
    %We are using some preview so stop running when we get near the end
    if ind+steps+1>length(theta)
        break;
    end
    
    %Are we on teh left or right side of the reference trajectory?
    side = sign(cos(theta(ind))*(x(k,3)-cline_y(ind))-sin(theta(ind))*(x(k,1)-cline_x(ind)));
    
    %Calculte the approximate perpendicular distance to the reference
    angle = atan2(x(k,3)-cline_y(ind),x(k,1)-cline_x(ind));
    r = abs(d*sin(angle-theta(ind)));
    
    %Gains, manually determined
    Kr = 0.1;
    Kh = 1;
    
    %Input
    u = [-side*Kr*r-Kh*(x(k,5)-theta(ind)) F_x];
    
    %Limit steering input
    if u(1) > 0.5
        u(1) = 0.5;
    end
    if u(1) < -0.5
        u(1) = -0.5;
    end
    
    %Go two steps forward
    step = twostep(u,x(k,:));
    x(k+1:k+steps,:) = step(end-steps+1:end,:);

    U(k:k+steps-1,:) = [u; u];  %YOU MUST CHANGE NUMBER OF "u" COLUMNS MANUALLY
    
    if rem(k,drift_control_step)==1
        Z = forwardIntegrateControlInput(U,x(1,:));  
        x(k+1:k+steps,:) = Z(end-steps+1:end,:);
    end

    k=k+steps;
end

calc_time=toc(ticid);
figure(h1)
plot(x(:,1),x(:,3),'-g');

%% Re-calculate trajectory with ode45 to gauge the difference
ticid_sim=tic;
x_final = forwardIntegrateControlInput(U,x(1,:));
figure(h1);
plot(x_final(:,1),x_final(:,3),'-.r');
sim_time=toc(ticid_sim)
%% Functions
function [Y] = twostep(U,x0)

% generate time vector
T = 0:0.01:0.02;    %YOU MUST CHANGE NUMBER OF TIME STEPS MANUALLY!

% constants
W = 13720 ;
Nw = 2 ;
f = 0.01 ;
Iz = 2667 ;
a = 1.35 ;
b = 1.45 ;
By = 0.27 ;
Cy = 1.2 ;
Dy = 2921 ;
Ey = -1.6 ;
Shy = 0 ;
Svy = 0 ;
m = 1400 ;

% slip angle functions in degrees
a_f = @(t,x) rad2deg(U(1)-atan2(x(4)+a*x(6),x(2))) ;
a_r = @(t,x) rad2deg(-atan2((x(4)-b*x(6)),x(2))) ;

% Nonlinear Tire Dynamics
phi_yf = @(t,x) (1-Ey)*(a_f(t,x)+Shy)+(Ey/By)*atan(By*(a_f(t,x)+Shy)) ;
phi_yr = @(t,x) (1-Ey)*(a_r(t,x)+Shy)+(Ey/By)*atan(By*(a_r(t,x)+Shy)) ;

F_yf = @(t,x) Dy*sin(Cy*atan(By*phi_yf(t,x)))+Svy ;
F_yr = @(t,x) Dy*sin(Cy*atan(By*phi_yr(t,x)))+Svy ;

% vehicle dynamics
df = @(t,x) [x(2)*cos(x(5))-x(4)*sin(x(5)) ;...
          (-f*W+Nw*U(2)-F_yf(t,x)*sin(U(1)))/m+x(4)*x(6) ;...
          x(2)*sin(x(5))+x(4)*cos(x(5)) ;...
          (F_yf(t,x)*cos(U(1))+F_yr(t,x))/m-x(2)*x(6) ;...
          x(6) ;...
          (F_yf(t,x)*a*cos(U(1))-F_yr(t,x)*b)/Iz] ;
      
% Solve for trajectory
[~,Y] = ode45(df,T,x0) ;

end
