function [df] = state_transition_euler(x,u)
% function [Y] = forwardIntegrateControlInput(U,x0)
% 
% Given a set of inputs and an initial condition, returns the vehicles
% trajectory. If no initial condition is specified the default for the track
% is used.
% 
%  INPUTS:
%    U           an N-by-2 vector of inputs, where the first column is the
%                steering input in radians, and the second column is the 
%                longitudinal force in Newtons.
%    
%    x0          a 1-by-6 vector of the initial state of the vehicle.
% 
%  OUTPUTS:
%    Y           an N-by-6 vector where each column is the trajectory of the
%                state of the vehicle
% 
%  Written by: Matthew Porter
%  Created: 13 Nov 2017
%  Modified: 11 Dec 2017 by Shreyas Kousik


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

% generate input functions
d_f = u(1);
F_x = u(2);

% slip angle functions in degrees
a_f = rad2deg(d_f-atan2(x(4)+a*x(6),x(2))) ;
a_r = rad2deg(-atan2((x(4)-b*x(6)),x(2))) ;

% Nonlinear Tire Dynamics
phi_yf =  (1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy)) ;
phi_yr =  (1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy)) ;

F_yf = Dy*sin(Cy*atan(By*phi_yf))+Svy ;
F_yr = Dy*sin(Cy*atan(By*phi_yr))+Svy ;

% vehicle dynamics
df = [x(2)*cos(x(5))-x(4)*sin(x(5)) ;...
          (-f*W+Nw*F_x-F_yf *sin(d_f))/m+x(4)*x(6) ;...
          x(2)*sin(x(5))+x(4)*cos(x(5)) ;...
          (F_yf *cos(d_f)+F_yr )/m-x(2)*x(6) ;...
          x(6) ;...
          (F_yf *a*cos(d_f)-F_yr *b)/Iz] ;
      
end
