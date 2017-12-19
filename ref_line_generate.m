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
%% Obstacle generation 
rng(42)
nobs=10;%no of obstacles
Xobs = generateRandomObstacles(nobs,TestTrack);
%% Reference input
% load('Control_Nom.mat')
% load('Trajectory_Nom.mat')
% u_ol=Control_Nom;
% x_ol=Y;
% clear Control_Nom Y
% load('traj_cline_slow.mat')
load('traj_cline_slow.mat')

%% Run open loop to get equilibrium trajectory
% x0 = [287 5 -176 0 2 0]';
% x_ol = forwardIntegrateControlInput(u_ol,x0);
t_sim=0:0.01:(size(u_ol,1)-1)*0.01;
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
    
    [indt1 indt2 indt3 indt4]
    pts=[xl_t1 xl_t2 xl_t3 xl_t4]
    
    ref_line(:,[indt1:indt4])=R_obs'*ref_line_rep_l;
    
    %Closest points on trajectory to obstacle edges
    for mmm=1:4
         [mindist,indcl]=sort(((obs_x_mat(mmm,pp)-x_ol(:,1)).^2+(obs_y_mat(mmm,pp)-x_ol(:,3)).^2),'ascend');
         cl_obs_traj(mmm,:)=[x_ol(indcl(1),1) x_ol(indcl(1),3)];
    end
        
   [in,on] = inpolygon(cl_obs_traj(:,1),cl_obs_traj(:,2),obs_x_mat(:,pp),obs_y_mat(:,pp));
   if (numel(cl_obs_traj(in,1))>0 || numel(cl_obs_traj(on,1))>0)
     avoid_obs(pp)=1;
   else
      avoid_obs(pp)=0;
   end 
    plot(Xobs{pp}(1:2,1),Xobs{pp}(1:2,2),'-r')
    plot(Xobs{pp}(2:3,1),Xobs{pp}(2:3,2),'-g')
    plot(Xobs{pp}(3:4,1),Xobs{pp}(3:4,2),'-b')
    plot(Xobs{pp}([4 1],1),Xobs{pp}([4 1],2),'-m')
    plot(obs_fm_mat(1,pp),obs_fm_mat(2,pp),'v')
    plot(obs_bm_mat(1,pp),obs_bm_mat(2,pp),'v')
    
end

figure(h1)
plot(ref_line(1,:),ref_line(2,:),'--k')