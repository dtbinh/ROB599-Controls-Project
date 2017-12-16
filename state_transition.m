function [x_next]=state_transition(x,u,dt)

%generate input functions
d_f=@(t) interp1(T,U(:,1),t,'previous','extrap');
F_x=@(t) interp1(T,U(:,2),t,'previous','extrap');

%slip angle functions in degrees
a_f=@(x,u) rad2deg(u(1)-atan2(x(4)+a*x(6),x(2)));
a_r=@(x,u) rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=@(x,u) (1-Ey)*(a_f(x,u)+Shy)+(Ey/By)*atan(By*(a_f(x,u)+Shy));
phi_yr=@(x,u) (1-Ey)*(a_r(x,u)+Shy)+(Ey/By)*atan(By*(a_r(x,u)+Shy));

F_yf=@(x,u) Dy*sin(Cy*atan(By*phi_yf(x,u)))+Svy;
F_yr=@(x,u) Dy*sin(Cy*atan(By*phi_yr(x,u)))+Svy;

%vehicle dynamics
df=@(x,u) [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*W+Nw*u(2)-F_yf(x,u)*sin(u(1)))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf(x,u)*cos(u(1))+F_yr(x,u))/m-x(2)*x(6);...
          x(6);...
          (F_yf(x,u)*a*cos(u(1))-F_yr(x,u)*b)/Iz];
      
%Next state
x_next=x+dt*f(x,u);

