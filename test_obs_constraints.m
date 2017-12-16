clear all
close all
clc
%%
load('Init_MPC_test.mat')

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

%%
x=x_ref';
u=u_ref';
k=60;
ind_det=[1 2];%Obstacle nos
pp=2;%Chosen obstacle
z_test=zeros(Ndec,1);
z_test=[reshape(x(:,k:k+horizon)-x_ref(k:k+horizon,:)',n*(horizon+1),1);reshape(u(:,k:k+horizon-1),m*(horizon),1)];

figure(h1)
plot(x(1,k),x(3,k),'sr')
        
    %Vehicle and obstacle points in obstacle C.S
    R_obs=[cos(th_obs(pp)) sin(th_obs(pp));-sin(th_obs(pp)) cos(th_obs(pp))];
    xvl=R_obs*[x(1,k);x(3,k)];
    xobl=R_obs*[obs_x_mat(:,ind_det(pp))';obs_y_mat(:,ind_det(pp))'];
    
    xv_test=R_obs*[x(1,k:k+horizon);x(3,k:k+horizon)];
                h2=figure
                plot(xv_test(1,:),xv_test(2,:),'-s')
%                 plot(xvl(1),xvl(2),'s')
                hold on
                plot([xvl(1) xobl(1,:)],[xvl(2) xobl(2,:)],'-')
                
    %             keyboard
    if (xvl(1)<max(xobl(1,:)))
        %Perpendicular distances from left side of obstacle to left side of track
        pdist1=perp_dist(obs_x_mat(1,ind_det(pp)),obs_y_mat(1,ind_det(pp)),TestTrack.bl);
        pdist4=perp_dist(obs_x_mat(4,ind_det(pp)),obs_y_mat(4,ind_det(pp)),TestTrack.bl);
        pdistl=min(pdist1,pdist4);
        %Perpendicular distances from right side of obstacle to right side of track
        pdist2=perp_dist(obs_x_mat(2,ind_det(pp)),obs_y_mat(2,ind_det(pp)),TestTrack.br);
        pdist3=perp_dist(obs_x_mat(3,ind_det(pp)),obs_y_mat(3,ind_det(pp)),TestTrack.br);
        pdistr=min(pdist2,pdist3);
        %Which side of obstacle is closest to car
        [mind2,indmin]=min((xvl(1)- obs_x_mat(1:2,ind_det(pp))).^2+(x(3,k)- obs_y_mat(1:2,ind_det(pp))).^2);
        
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
        
        %Get slopes and y-intercepts of constraining lines
        if (goleft==1) %Going left
            if ( xvl(1)<xobl(1,1)) %if car is approximately  in front of obstacle
                m_cons=(xobl(2,1)-xvl(2))/(xobl(1,1)-xvl(1));
                c_cons=xvl(2)-m_cons*xvl(1);
                flag_cons=1;
            elseif (xvl(1)>xobl(1,1) && xvl(1)<xobl(1,4)) %if car is approximately to the side of obstacle
                m_cons=(xobl(2,4)-xobl(2,1))/(xobl(1,4)-xobl(1,1));
                c_cons=xobl(2,1)-m_cons*xobl(1,1);
                flag_cons=1;
            else
                flag_cons=0;
            end
            
        else %Going right
            if(xvl(1)<xobl(1,2)) %if car is approximately  in front of obstacle
                m_cons=(xobl(2,2)-xvl(2))/(xobl(1,2)-xvl(1));
                c_cons=xvl(2)-m_cons*xvl(1);
                flag_cons=1;
            elseif (xvl(1)>xobl(1,2) && xvl(1)<xobl(1,3)) %if car is approximately to the side of obstacle
                m_cons=(xobl(2,3)-xobl(2,2))/(xobl(1,3)-xobl(1,2));
                c_cons=xobl(2,2)-m_cons*xobl(1,2);
                flag_cons=1;
            else
                flag_cons=0;
            end
        end
        
        if (flag_cons==1)
            Aex=zeros(horizon,Ndec);
            bex=zeros(horizon,1);
            if (goleft==1)
                %Extra constraint equations for going left
                for lll=1:horizon
                    Aex(lll, lll*n+[1 3])=[m_cons -1]*R_obs;
                    bex(lll,1)=-c_cons-[m_cons -1]*R_obs*[x_ref(k+lll,1);x_ref(k+lll,3)];
                end
            else
                %Extra constraint equations for going right
                for lll=1:horizon
                    Aex(lll, lll*n+[1 3])=-[m_cons -1]*R_obs;
                    bex(lll,1)=c_cons+[m_cons -1]*R_obs*[x_ref(k+lll,1); x_ref(k+lll,3)];
                end
            end
        end
    end

    goleft
        
    Aex*z_test-bex
flag_cons