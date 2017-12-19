clear all
close all
clc
%% Load test data
load('Init_MPC_test.mat')
close all
%%
h1=figure;
hold on
plot(bl_x,bl_y,br_x,br_y,'b');%cline_x,cline_y,'--',
for pp=1:nobs
    plot(Xobs{pp}(1:2,1),Xobs{pp}(1:2,2),'-r')
    plot(Xobs{pp}(2:3,1),Xobs{pp}(2:3,2),'-g')
    plot(Xobs{pp}(3:4,1),Xobs{pp}(3:4,2),'-b')
    plot(Xobs{pp}([4 1],1),Xobs{pp}([4 1],2),'-m')
    plot(obs_fm_mat(1,pp),obs_fm_mat(2,pp),'v')
    plot(obs_bm_mat(1,pp),obs_bm_mat(2,pp),'v')
end
figure(h1);
hold on
plot(x_ref(:,1),x_ref(:,3),'-.m');

%% Testing parameters
x=x_ref';
u=u_ref';
k=10;%60;%Time index for testing
% n_det=1;%Obstacle detected
pp=1;%2;%Chosen obstacle
z_test=zeros(Ndec,1);
z_test(1:3)=[266 0  -118]-x_ref(k,1:3);%[242 0  -49]-x_ref(k,1:3);
z_test(n*horizon+[1:3])=[264 0  -112]-x_ref(k+horizon,1:3);%[ 244 0 -50]-x_ref(k+horizon,1:3);
x(:,k:k+horizon)=transpose(reshape(z_test(1:n*(horizon+1)),n,horizon+1)'+x_ref(k:k+horizon,:));
% z_test=[reshape(x(:,k:k+horizon)-x_ref(k:k+horizon,:)',n*(horizon+1),1);reshape(u(:,k:k+horizon-1),m*(horizon),1)];
%%
horizon_def=5;    
figure(h1)
        plot(x(1,[k k+horizon]),x(3,[k k+horizon]),'-sr')
        plot([x(1,k) obs_x_mat(1,pp)], [x(3,k) obs_y_mat(1,pp)],'-')
        plot([x(1,k) obs_x_mat(2,pp)], [x(3,k) obs_y_mat(2,pp)],'-')
go_left_mat=[];

 %Detect how many obstacles are close at initial time
    [d2_det,ind_det]=find(((x(1,k)- obs_fm_mat(1,:)).^2+(x(3,k)- obs_fm_mat(2,:)).^2)<detect_dist^2);%Index of obstacles which are close enough
    
    n_det=length(ind_det); %No of closest obstacles detected
    
    if (n_det>0) %If obstacle is detected change horizon
        %Index of closest obstacle
        [mind2_obs,pp]=min((x(1,k)- obs_fm_mat(1,:)).^2+(x(3,k)- obs_fm_mat(2,:)).^2);
        
        %Vehicle and obstacle points in obstacle C.S
        R_obs=[cos(th_obs(pp)) sin(th_obs(pp));-sin(th_obs(pp)) cos(th_obs(pp))];;%Rotation matrix to get from world CS to obstacle CS
        xvl=R_obs*[x(1,k);x(3,k)];
        xobl=R_obs*[obs_x_mat(:,pp)';obs_y_mat(:,pp)'];
        %Change horizon only id closest obstacle is in front
        if (xvl(1)<max(xobl(1,:)))
%         horizon = 20;%ceil(detect_dist/(dt*norm(mean(x_ref(:,[2 4])))));%No of steps to reach obstacle approximately 
        else
        horizon=horizon_def;    
        end
    else
       horizon=horizon_def;
        
    end
  
    
    %Inequality constraints for obstacles
   
    if n_det>0
        
        %Index of closest obstacle
%         [mind2_obs,pp]=min((x(1,k)- obs_fm_mat(1,:)).^2+(x(3,k)- obs_fm_mat(2,:)).^2);
%         
% %         for pp=1:n_det %Loop over obstacles wthin range
%             %Vehicle and obstacle points in obstacle C.S
%             R_obs=[cos(th_obs(pp)) sin(th_obs(pp));-sin(th_obs(pp)) cos(th_obs(pp))];;%Rotation matrix to get from world CS to obstacle CS
%             xvl=R_obs*[x(1,k);x(3,k)];
%             xobl=R_obs*[obs_x_mat(:,pp)';obs_y_mat(:,pp)'];
            xvl_h=R_obs*[x(1,k+horizon);x(3,k+horizon)];
            h2=figure
            plot([xvl(1) xvl_h(1)],[xvl(2) xvl_h(2)],'-s')
            hold on
            plot(xobl(1,:),xobl(2,:),'-')
            plot([xvl(1) xobl(1,1)],[xvl(2) xobl(2,1)],'-')
            plot([xvl(1) xobl(1,2)],[xvl(2) xobl(2,2)],'-')
%             keyboard
            if (xvl(1)<max(xobl(1,:)))
                
                %Perpendicular distances from left side of obstacle to left side of track
                pdist1=perp_dist(obs_x_mat(1,pp),obs_y_mat(1,pp),TestTrack.bl);
                pdist4=perp_dist(obs_x_mat(4,pp),obs_y_mat(4,pp),TestTrack.bl);
                pdistl=min(pdist1,pdist4);
                %Perpendicular distances from right side of obstacle to right side of track
                pdist2=perp_dist(obs_x_mat(2,pp),obs_y_mat(2,pp),TestTrack.br);
                pdist3=perp_dist(obs_x_mat(3,pp),obs_y_mat(3,pp),TestTrack.br);
                pdistr=min(pdist2,pdist3);
                %Which side of obstacle is closest to car
                [mind2,indmin]=min((x(1,k)- obs_x_mat(1:2,pp)).^2+(x(3,k)- obs_y_mat(1:2,pp)).^2);
                
                if (indmin==1 && pdistl>lim_pdist) %left side is closer and there is space to go on that side
                    goleft=1;
                elseif (indmin==2 && pdistr>lim_pdist) %right side is closer and there is space to go on that side
                    goleft=0;
                else
                    if (pdistl>pdistr)%More space on left side even if its lower than limit
                        goleft=1;
                    else %More space on left side even if its lower than limit
                        goleft=0;
                    end
                end
                go_left_mat=[go_left_mat goleft];
                %Get slopes and y-intercepts of constraining lines
                if (goleft==1) %Going left
                    if ( xvl(1)<xobl(1,1)) %if car is approximately  in front of obstacle
                        m_cons=(xobl(2,1)-xvl(2))/(xobl(1,1)-xvl(1));
                        c_cons=xvl(2)-m_cons*xvl(1);
                        flag_cons=1;
                        out=1
                    elseif (xvl(1)>xobl(1,1) && xvl(1)<xobl(1,4)) %if car is approximately to the side of obstacle
                        m_cons=(xobl(2,4)-xobl(2,1))/(xobl(1,4)-xobl(1,1));
                        c_cons=xobl(2,1)-m_cons*xobl(1,1);
                        flag_cons=1;
                        out=2
                    else
                        flag_cons=0;
                        out=3
                    end
                    
                else %Going right
                    if( xvl(1)<xobl(1,2)) %if car is approximately  in front of obstacle
                        m_cons=(xobl(2,2)-xvl(2))/(xobl(1,2)-xvl(1));
                        c_cons=xvl(2)-m_cons*xvl(1);
                        flag_cons=1;
                        out=4
                    elseif (xvl(1)>xobl(1,2) && xvl(1)<xobl(1,3)) %if car is approximately to the side of obstacle
                        m_cons=(xobl(2,3)-xobl(2,2))/(xobl(1,3)-xobl(1,2));
                        c_cons=xobl(2,2)-m_cons*xobl(1,2);
                        flag_cons=1;
                        out=5
                    else
                        flag_cons=0;
                        out=6
                    end
                end
                
                if (flag_cons==1)
                     Aex=zeros(1,Ndec);%zeros(horizon,Ndec);
                     bex=zeros(1,1);
                    if (goleft==1)
                        %Extra constraint equations for going left
                            Aex(1, horizon*n+[1 3])=[m_cons -1]*R_obs;
                            bex(1,1)=-c_cons-[m_cons -1]*R_obs*[x_ref(k+horizon,1);x_ref(k+horizon,3)];
%                         for lll=1:horizon
%                             Aex(lll, lll*n+[1 3])=[m_cons -1]*R_obs;
%                             bex(lll,1)=-c_cons-[m_cons -1]*R_obs*[x_ref(k+lll,1);x_ref(k+lll,3)];
%                         end
                    else
                        %Extra constraint equations for going right
                        Aex(1, horizon*n+[1 3])=-[m_cons -1]*R_obs;
                        bex(1,1)=c_cons+[m_cons -1]*R_obs*[x_ref(k+horizon,1);x_ref(k+horizon,3)];
%                         for lll=1:horizon
%                             Aex(lll, lll*n+[1 3])=-[m_cons -1]*R_obs;
%                             bex(lll,1)=c_cons+[m_cons -1]*R_obs*[x_ref(k+lll,1); x_ref(k+lll,3)];
%                         end
                    end
%                 Aineq=[Aineq;Aex];
%                 bineq=[bineq;bex];
                end
                flag_detect=1;
            else
                flag_detect=0;
            end
%         end
    end
 figure(h1)
 xlim([x(1,k)-10 x(1,k+horizon)+10])
 ylim([x(3,k)-10 x(3,k+horizon)+10])
%% Check stuff
out
Aex*z_test-bex%Test if last state at end of horizon is passing(constraint<0) or failing(constraint>0) 
go_left_mat
flag_cons