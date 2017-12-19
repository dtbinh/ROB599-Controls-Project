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

%% Obstacle generation and collision detection
rng(42)
nobs=10;%no of obstacles
Xobs = generateRandomObstacles(nobs,TestTrack);
%%
h1=figure;
hold on
plot(bl_x,bl_y,br_x,br_y,'b');%cline_x,cline_y,'--',
for pp=1:nobs
    obs_x_mat(:,pp)=Xobs{pp}(1:4,1);
    obs_y_mat(:,pp)=Xobs{pp}(1:4,2);
    obs_c_mat(:,pp)=[mean(Xobs{pp}(1:4,1));mean(Xobs{pp}(1:4,2))];
    obs_fm_mat(:,pp)=[mean(Xobs{pp}(1:2,1));mean(Xobs{pp}(1:2,2))];
    obs_bm_mat(:,pp)=[mean(Xobs{pp}(3:4,1));mean(Xobs{pp}(3:4,2))];
    th_obs(pp)=atan2(obs_bm_mat(2,pp)- obs_fm_mat(2,pp),obs_bm_mat(1,pp)- obs_fm_mat(1,pp));
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
%%

%Initialization
detect_dist=5;%Detection distance of obstacle in m
lim_pdist=1.5;%Limiting distance on side of obstacleto considet going to far side
dt = 0.1;
t=0:dt:floor(t_sim(end)/dt)*dt;%500*dt;%117.5;

x_ref=interp1(t_sim,x_ol,t,'previous');
u_ref=interp1(t_sim,u_ol,t,'previous');

figure(h1);
hold on
plot(x_ref(:,1),x_ref(:,3),'-.m');

x = x_ref(1,:)';% [289 5 -175 0 2 0]';%[287 5 -176 0 2 0]';
e = x-x_ref(1,:)';
u = u_ref(1,:);%[-0.02 5000];


%Normal parameters
err_UB=[200 20 200 20 1 1];%Error upper bounds
err_LB=-err_UB;%Error lower bounds
u_lim=[-0.2 0.2;-1000 5000];%Total input bounds
obs_pad=1;%Obstacle padding in m
t_comp_mins=15;%No. of simulation minutes allowed for competition

n = 6;                    %Number of States
m = 2;                    %Number of Inputs
horizon=2;
horizon_def=horizon;
zsize = (horizon+1)*n+horizon*m;
xsize = (horizon+1)*n;
Q = diag(repmat((ones(1,n)./[100 10 100 10 0.1 0.01]).^2,1,horizon+1));%eye(xsize);%
R = zeros(zsize-xsize);%diag(repmat([1 1]./[0.01 1000],1,horizon));%
H = blkdiag(Q, R);
f = zeros(zsize, 1);
Ndec=n * (horizon+1) + m *horizon ;
options=optimoptions(@fmincon,'Algorithm','sqp','SpecifyObjectiveGradient',true);%'SpecifyConstraintGradient',true);%'CheckGradients',true);%,'Display','Iter');%,'CheckGradients',false,'GradObj','on','ConstraintTolerance',1e-6,'MaxIter',10000,'MaxFunctionEvaluations',50000,'Display','Iter');%,[repmat([100;5;100;5;1;0.1],(horizon+1),1) ; repmat([0.1;1000],horizon,1)]'TypicalX',sqrt(1./diag(Q))

go_left_mat=[];
exitflag_mat=[];
recover_norm=1e-4;

MPC_flag=0;
k=0;
pp=[];
exit_flag=0;

%MPC at every time step k
while exit_flag==0
    k=k+1;
    ((norm(x(:,k)-x_ref(k,:)')/norm(x_ref(k,:)'))<recover_norm)
    MPC_flag
    if ((k==(length(t)-1)) || toc(ticid)>=0.95*t_comp_mins*60) %
        exit_flag=1;
    end
    disp(['At ' num2str(t(k)) ' s:'])
    %Detect how many obstacles are close at initial time
    [d2_det,ind_det]=find(((x(1,k)- obs_fm_mat(1,:)).^2+(x(3,k)- obs_fm_mat(2,:)).^2)<detect_dist^2);%Index of obstacles which are close enough
    
    n_det=length(ind_det); %No of closest obstacles detected
    
    if (n_det>0) %If obstacle is detected change horizon
        %Index of closest obstacle
        [mind2_obs,pp]=min((x(1,k)- obs_fm_mat(1,:)).^2+(x(3,k)- obs_fm_mat(2,:)).^2);
        
        %Vehicle and obstacle points in obstacle C.S
        R_obs=[cos(th_obs(pp)) sin(th_obs(pp));-sin(th_obs(pp)) cos(th_obs(pp))];%Rotation matrix to get from world CS to obstacle CS
        xvl=R_obs*[x(1,k);x(3,k)];
        xobl=R_obs*[obs_x_mat(:,pp)';obs_y_mat(:,pp)'];
        %Change horizon only id closest obstacle is in front and it clashes
        %with trajectory
        if (xvl(1)<=max(xobl(1,:)) && (avoid_obs(pp)==1) && (MPC_flag==0))
             MPC_flag=1;
             horizon = 3;%ceil(detect_dist/(dt*norm(mean(x_ref(:,[2 4])))));%No of steps to reach obstacle approximately
        elseif (xvl(1)>max(xobl(1,:)) && (avoid_obs(pp)==1))
            horizon=horizon_def;
            if (((norm(x(:,k)-x_ref(k,:)')/norm(x_ref(k,:)'))<recover_norm) && (MPC_flag==1))
            MPC_flag=0;
            end
        end
    else
        horizon=horizon_def;
        if (((norm(x(:,k)-x_ref(k,:)')/norm(x_ref(k,:)'))<recover_norm) && (MPC_flag==1))
            MPC_flag=0;
        end
    end
    
    if (MPC_flag==1)
        if ~(length(t)-k>horizon)
            horizon = length(t)-k;
        end
        
        zsize = (horizon+1)*n+horizon*m;
        xsize = (horizon+1)*n;
        Q = diag(repmat((ones(1,n)./[100 10 100 10 0.1 0.01]).^2,1,horizon+1));%1000*eye(xsize);
        R = zeros(zsize-xsize);
        H = blkdiag(Q, R);
        f = zeros(zsize, 1);
        Ndec=n * (horizon+1) + m *horizon ;
        
        fun=@(var)error_cost(var,H,f);
        %     nlcons=@(var)track_nlcons_err(var,reshape(x_ref(k:k+horizon,:)',n*(horizon+1),1),TestTrack,Ndec,horizon,n,m);%Nonlinear track constraints
        
        %Evaluate Al and Bl at current state
        [Al,Bl]=linearized_mats(x_ref(k,:),u_ref(k,:));
        sysc=ss(Al,Bl,[],[]);
        %Discretize (Euler)
%         A = eye(size(Al))+dt*Al;
%         B = dt*Bl;
         sysd=c2d(sysc,dt,'zoh');
         A=sysd.A;
        B=sysd.B;
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
        
        %
        %% Inequality constraints
        
        %Inequality constraints for input limits
        %Upper Bounds
        Aineq = zeros(2*Ndec, Ndec);
        bineq = zeros(2*Ndec, 1);
        Aineq(1:n * (horizon+1),1:n * (horizon+1))=eye(n * (horizon+1));
        bineq(1:n * (horizon+1),1)=repmat(err_UB',(horizon+1),1);
        Aineq(n * (horizon+1)+[1:m*horizon],n * (horizon+1)+[1:m*horizon])=eye(m*horizon);
        bineq(n * (horizon+1)+[1:2:m*horizon],1)=u_lim(1,2)-u_ref(k:k+horizon-1,1);
        bineq(n * (horizon+1)+[2:2:m*horizon],1)=u_lim(2,2)-u_ref(k:k+horizon-1,2);
        
        %Lower Bounds
        Aineq(Ndec+[1:n * (horizon+1)],1:n * (horizon+1))=-eye(n * (horizon+1));
        bineq(Ndec+[1:n * (horizon+1)],1)=repmat(-err_LB',(horizon+1),1);
        Aineq(Ndec+n * (horizon+1)+[1:m*horizon],n * (horizon+1)+[1:m*horizon])=-eye(m*horizon);
        bineq(Ndec+n * (horizon+1)+[1:2:m*horizon],1)=u_lim(1,2)-u_ref(k:k+horizon-1,1);
        bineq(Ndec+n * (horizon+1)+[2:2:m*horizon],1)=u_lim(2,2)-u_ref(k:k+horizon-1,2);
        
        %Inequality constraints for slip angle limits
        Asl = zeros(4*horizon, Ndec);
        bsl = zeros(4*horizon,1);
        for lll=1:horizon
            Asl(lll,lll*n+[2 4 6])=[-0.3 1 1.35];
            Asl(horizon+lll,lll*n+[2 4 6])=[-0.3 -1 -1.35 ];
            Asl(2*horizon+lll,lll*n+[2 4 6])=[-0.05 1 -1.45];
            Asl(3*horizon+lll,lll*n+[2 4 6])=[-0.05 -1 1.45 ];
        end
        
        Aineq= [Aineq; Asl];
        bineq= [bineq;bsl];
        
        %Inequality constraints for input limits
        %     Aineq = zeros( 2*m*horizon, Ndec);
        %     bineq = zeros( 2*m*horizon, 1 );
        %     Aineq(1:m*horizon,n * (horizon+1)+[1:m*horizon])=eye(m*horizon);
        %     bineq([1:2:m*horizon],1)=u_lim(1,2)-u_ref(k:k+horizon-1,1);
        %     bineq([2:2:m*horizon],1)=u_lim(2,2)-u_ref(k:k+horizon-1,2);
        
        %     Aineq(m*horizon+[1:m*horizon],n * (horizon+1)+[1:m*horizon])=-eye(m*horizon);
        %     bineq(m*horizon+[1:2:m*horizon],1)=-u_lim(1,1)+u_ref(k:k+horizon-1,1);
        %     bineq(m*horizon+[2:2:m*horizon],1)=-u_lim(2,1)+u_ref(k:k+horizon-1,2);
        %
        %Inequality constraints for obstacles
        
        if n_det>0
            figure(h1)
            plot(x(1,k),x(3,k),'sr')
            %Index of closest obstacle
            %         [mind2_obs,pp]=min((x(1,k)- obs_fm_mat(1,:)).^2+(x(3,k)- obs_fm_mat(2,:)).^2);
            %
            % %         for pp=1:n_det %Loop over obstacles wthin range
            %             %Vehicle and obstacle points in obstacle C.S
            %             R_obs=[cos(th_obs(pp)) sin(th_obs(pp));-sin(th_obs(pp)) cos(th_obs(pp))];;%Rotation matrix to get from world CS to obstacle CS
            %             xvl=R_obs*[x(1,k);x(3,k)];
            %             xobl=R_obs*[obs_x_mat(:,pp)';obs_y_mat(:,pp)'];
            %             h2=figure
            %             plot(xvl(1),xvl(2),'s')
            %             hold on
            %             plot(xobl(1,:),xobl(2,:),'-')
            % %             keyboard
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
                        m_cons=((xobl(2,1)+obs_pad)-xvl(2))/(xobl(1,1)-xvl(1));
                        c_cons=xvl(2)+obs_pad-m_cons*xvl(1);
                        flag_cons=1;
                    elseif (xvl(1)>xobl(1,1) && xvl(1)<xobl(1,4)) %if car is approximately to the side of obstacle
                        m_cons=(xobl(2,4)-xobl(2,1))/(xobl(1,4)-xobl(1,1));
                        c_cons=xobl(2,1)+obs_pad-m_cons*xobl(1,1);
                        flag_cons=1;
                    else
                        flag_cons=0;
                    end
                    
                else %Going right
                    if( xvl(1)<xobl(1,2)) %if car is approximately  in front of obstacle
                        m_cons=(xobl(2,2)-obs_pad-xvl(2))/(xobl(1,2)-xvl(1));
                        c_cons=xvl(2)-obs_pad-m_cons*xvl(1);
                        flag_cons=1;
                    elseif (xvl(1)>xobl(1,2) && xvl(1)<xobl(1,3)) %if car is approximately to the side of obstacle
                        m_cons=(xobl(2,3)-xobl(2,2))/(xobl(1,3)-xobl(1,2));
                        c_cons=xobl(2,2)-obs_pad-m_cons*xobl(1,2);
                        flag_cons=1;
                    else
                        flag_cons=0;
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
                    Aineq=[Aineq;Aex];
                    bineq=[bineq;bex];
                end
                flag_detect=1;
            else
                flag_detect=0;
            end
            %         end
        end
        
        %Inequality constraints (approximate linear) for track
        %Find closest points on track
        [mindist_tr,ind_cl_tr]=sort(((x(1,k)-TestTrack.cline(1,:)).^2+(x(3,k)-TestTrack.cline(2,:)).^2),'ascend');
        ind_tr=[min(ind_cl_tr(1:2)) max(ind_cl_tr(1:2))];
        %Locations of closest centerline points
        xc1=TestTrack.cline(1,ind_tr(1)); yc1=TestTrack.cline(2,ind_tr(1));
        xc2=TestTrack.cline(1,ind_tr(2)); yc2=TestTrack.cline(2,ind_tr(2));
        
        th_tr=atan2((yc2-yc1),(xc2-xc1));%Slope of centerline segment
        R_tr=[cos(th_tr) sin(th_tr);-sin(th_tr) cos(th_tr)];%Rotation matrix to get from world CS to track segment CS
        
        %Locations of closest left track points
        xl1=TestTrack.bl(1,ind_tr(1)); yl1=TestTrack.bl(2,ind_tr(1));
        xl2=TestTrack.bl(1,ind_tr(2)); yl2=TestTrack.bl(2,ind_tr(2));
        %Locations of closest right track points
        xr1=TestTrack.br(1,ind_tr(1)); yr1=TestTrack.br(2,ind_tr(1));
        xr2=TestTrack.br(1,ind_tr(2)); yr2=TestTrack.br(2,ind_tr(2));
        
        %Express in track C.S
        xc_tr=R_tr*[xc1 xc2;yc1 yc2];
        xr_tr=R_tr*[xr1 xr2;yr1 yr2];
        xl_tr=R_tr*[xl1 xl2;yl1 yl2];
        xv_tr=R_tr*[x(1,k);x(3,k)];
        
        Aex2=zeros(2*horizon,Ndec);
        bex2=zeros(2*horizon,1);
        for lll=1:horizon
            Aex2(lll,lll*n+[1 3])=R_tr(2,:);%y-error for vehicle in track C.S
            Aex2(horizon+lll,lll*n+[1 3])=-R_tr(2,:);%negative y-error for vehicle in track C.S
            bex2(lll,1)=min(xl_tr(2,:))-0.1-R_tr(2,:)*[x_ref(k+lll,1);x_ref(k+lll,3)];%closest point of track on left side in in track C.S
            bex2(horizon+lll,1)=-(max(xr_tr(2,:))+0.1)+R_tr(2,:)*[x_ref(k+lll,1);x_ref(k+lll,3)];%closest point of track on right side in in track C.S
        end
        
        Aineq=[Aineq;Aex2];
        bineq=[bineq;bex2];
        %     figure(3)
        %     hold on
        %     plot(R_tr(1,:)*[x(1,k);x(3,k)],R_tr(2,:)*[x(1,k);x(3,k)],'s')
        %     plot(xc_tr(1,:),xc_tr(2,:),'g')
        %     plot(xr_tr(1,:),xr_tr(2,:),'b')
        %     plot(xl_tr(1,:),xl_tr(2,:),'--k')
        %     plot(xl_tr(1,:),( bex2(1,1))*ones(2,1),'--m')
        %     plot(xr_tr(1,:),-(bex2(horizon+1,1))*ones(2,1),'--m')
        %     keyboard
        
        % Inequality constraints to make sure car is moving forward
        Aex3=zeros(horizon,Ndec);
        bex3=zeros(horizon,1);
        for lll=1:horizon
            Aex3(lll,(lll-1)*n+[1 3])=R_tr(1,:);%x-error for vehicle in track C.S at lll-1=k:k+horizon-1
            Aex3(lll,lll*n+[1 3])=R_tr(1,:);%x-error for vehicle in track C.S at lll=k:k+horizon
            bex3(lll,1)=R_tr(1,:)*([x_ref(k+lll,1);x_ref(k+lll,3)]-[x_ref(k+lll-1,1);x_ref(k+lll-1,3)]);%Difference between references in track C.S
        end
        
        Aineq=[Aineq;Aex3];
        bineq=[bineq;bex3];
        %% Simulate open loop for initial guess
        z0=zeros(Ndec,1);
        e_curr=x(:,k)-x_ref(k,:)';
        z0([1:n])=e_curr;
        for ll=2:horizon+1
            e_curr=A*e_curr;%+B*u(k,:)';
            z0((ll-1)*n+[1:n])=e_curr;
        end
        %% Minimize
        %     z0(n*(horizon+1)+[1:m*horizon])=repmat(u(k,:)',1,horizon);
        [z,fcostval,exitflag,output] = fmincon(fun,z0,Aineq,bineq,Aeq,beq,[],[],[],options);
        exitflag_mat=[exitflag_mat exitflag];
        u_k = z([xsize+1 xsize+2],1);
        
        u(k+1,:) = u_k'+u_ref(k,:); %augmenting with nominal input
            e= [e z([n+1:2*n],1)];
            x=[x x_ref(k+1,:)'+z([n+1:2*n],1)]; %updating states
    %     Simulating using the total input
%     [L]=forwardIntegrate_vardt([u(k:k+1,:)],x(:,k)',dt);
%     
%     x=[x L(2,:)']; %updating states
%     e= [e L(2,:)'-x_ref(k+1,:)'];
        
    else
        u(k+1,:) = u_ref(k,:); %augmenting with nominal input
        e= [e zeros(n,1)];
        x=[x x_ref(k+1,:)']; %updating states
    end
    %%
    figure(h1)
    hold on
    plot([x(1,k) x(1,k+1)],[x(3,k) x(3,k+1)],'--g')
    
    %     if exist('flag_cons')
    %         if (flag_cons==1)
    %             if max(Aex*z-bex)>0
    %             keyboard
    %             end
    %         end
    %     end
    flag_cons=[];
    Aex=[];
    bex=[];
    Aex2=[];
    bex2=[];
end
%%
figure;
plot(t(1:k+1),e(:,1:k+1))
legend('e_x','e_u','e_y','e_v','e_{\psi}','e_{r}')
%% Re-interpolate final input vector
u_final=interp1(t(1:k+1),u(1:k+1,:),t_sim(t_sim<=t(k+1)),'previous',0);
u_final=[u_final;u_ol(t_sim>t(k+1),:)];
calc_time=toc(ticid)
figure;
subplot(211)
hold on
plot(u_ol(:,1));plot(u_final(:,1),'--r')
subplot(212)
hold on
plot(u_ol(:,2));plot(u_final(:,2),'--r')


%% Re-calculate trajectory with new input
ticid_sim=tic;
x_final = forwardIntegrateControlInput(u_final,x(:,1));
figure(h1);
plot(x_final(:,1),x_final(:,3),'--k');
sim_time=toc(ticid_sim)
% save('quadpog_obstacle_avoid_40s.mat')