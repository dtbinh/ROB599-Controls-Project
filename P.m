%P control
clear all
clc

load('TestTrack.mat')

bl_x = TestTrack.bl(1,:);
bl_y = TestTrack.bl(2,:);

br_x = TestTrack.br(1,:);
br_y = TestTrack.br(2,:);

cline_x = TestTrack.cline(1,:);
cline_y = TestTrack.cline(2,:);

theta = TestTrack.theta(1,:);

x(1,:) = [287 5 -176 0 2 0];


for i = 1:40000
    
    %Stay as close to the desired max speed as possible
    if x(i,2)>10
        F_x = 0;
    else
        F_x = 200;
    end
    
    %Determine nearest point of reference trajectory
    [ind, d] = knnsearch([cline_x; cline_y]',[x(i,1) x(i,3)]);
    
    %Are we on teh left or right side of the reference trajectory?
    side = sign(cos(theta(ind))*(x(i,3)-cline_y(ind))-sin(theta(ind))*(x(i,1)-cline_x(ind)));
    
    %Calculte the approximate perpendicular distance to the reference
    angle = atan2(x(i,3)-cline_y(ind),x(i,1)-cline_x(ind));
    r = abs(d*sin(angle-theta(ind)));
    
    %Gains, manually determined
    Kr = 0.02;
    Kh = 1;
    
    %We are using some preview so stop running when we get near the end
    if ind+1>length(theta)
        break;
    end
    
    %Input
    u = [-side*Kr*r-Kh*(x(i,5)-theta(ind+1)) F_x];
    
    %Limit steering input
    if u(1) > 0.5
        u(1) = 0.5;
    end
    if u(1) < -0.5
        u(1) = -0.5;
    end
    
    %Go one step forward
    step = onestep(u,x(i,:));
    x(i+1,:) = step(end,:);

    U(i,:) = u;
    
    %Stop if we are obviously off the track
    if r > 6
        break
    end

end

plot(bl_x,bl_y,br_x,br_y,cline_x,cline_y)
hold on
plot(x(:,1),x(:,3))

%The code below is just a modified version of forwardIntegrateControlInput
%The only difference being it only goes 1 step forward
function [Y] = onestep(U,x0)

% generate time vector
T = [0 0.01];

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

