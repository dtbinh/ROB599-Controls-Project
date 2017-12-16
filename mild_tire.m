clear all
close all
clc
%%
%Vehicle Parameters
a   =  1.14;		% distance c.g. to front axle (m) 
L   =  2.54;		% wheel base (m)
m   =  1500;		% mass (kg)
Iz  =  2420.0;	% yaw moment of inertia (kg-m^2)
b=L-a;   %distance of c.g to rear axel (m) 
g=9.81;
vx=20;

%%Tire forces
B=10;
C=1.3;
D=1;
E=0.97;


%timespan for all simulations
T=0:0.01:1;

%1.1 compute front and rear cornerning stifness
Ca_r=a/L*m*g*B*C*D;
Ca_f=b/L*m*g*B*C*D;
% 
%1.2.1 compute the front and rear cornering stifness for the vehicle generate equilibrium trajetory using Euler integration and linear tire
%forces

delta_fun=@(t) pi/180*sin(2*pi*t)-0.00175;


Z_eq = zeros(length(T),5);

    Z_eq(1,:) = zeros(5,1)';
    
    for i = 1:length(T)-1
        al_f(i)=delta_fun(T(i))-((Z_eq(i,4)+a*Z_eq(i,5))/vx);
        al_r(i)=-((Z_eq(i,4)-b*Z_eq(i,5))/vx);
        F_yf(i)=Ca_f*al_f(i);
        F_yr(i)=Ca_r*al_r(i);
        A_Z=[vx*cos(Z_eq(i,3))-Z_eq(i,4)*sin(Z_eq(i,3));
            Z_eq(i,4)*cos(Z_eq(i,3))+vx*sin(Z_eq(i,3));
            Z_eq(i,5);
            (1/m)*(F_yr(i)+F_yf(i)-m*vx*Z_eq(i,5));
            (1/Iz)*(-b*F_yr(i)+a*F_yf(i))];
        Z_eq(i+1,:) = Z_eq(i,:) + (T(i+1) - T(i))*A_Z';
    end
        al_f(i+1)=delta_fun(T(i+1))-((Z_eq(i+1,4)+a*Z_eq(i+1,5))/vx);
        al_r(i+1)=-((Z_eq(i+1,4)-b*Z_eq(i+1,5))/vx);
        F_yf(i+1)=Ca_f*al_f(i+1);
        F_yr(i+1)=Ca_r*al_r(i+1);
Z_eq=Z_eq';


% %1.2.2 linearization for feedback gains bike with linear tire forces
A1=@(i) [0 cos(psi) 0  -1 -sin(psi) -u*sin(psi)-v*cos(psi) 0  ;
       0 0 -Z_eq(4,i)*sin(Z_eq(3,i))+vx*cos(Z_eq(3,i)) cos(Z_eq(3,i)) 0;
       0 0 0 0 1;
       0 0 0 (1/m)*(-(Ca_r/vx)-(Ca_f/vx)) (1/m)*((Ca_r*b/vx)-(Ca_f*a/vx)-m*vx);
       0 0 0 (1/Iz)*((b*Ca_r/vx)-(a*Ca_f/vx)) (1/Iz)*((-Ca_r*b^2/vx)-(Ca_f*a^2/vx))];
         
B1=@(i) [0; 0 ;0 ; Ca_f/m ;(a*Ca_f)/Iz];
Q=eye(5);
R=0.5;        
[K,P]=lqr_LTV(A1,B1,Q,R,T);

% %1.2.3 Plot linear vs nonlinear tire forces and find max % difference
F_yf_pjk=(b/L)*m*g*D*sin(C*atan(B*(1-E)*al_f+E*atan(B*al_f)));
F_yr_pjk=(a/L)*m*g*D*sin(C*atan(B*(1-E)*al_r+E*atan(B*al_r)));
err_f=max(abs(F_yf-F_yf_pjk)./abs(F_yf_pjk))*100
err_r=max(abs(F_yr-F_yr_pjk)./abs(F_yr_pjk))*100
tireforce_percent_error=max(err_r,err_f);

% %1.2.4 Euler Simulate with Nonlinear tire dynamics
Z_nl = zeros(length(T),5);
delta_lim=[-45*pi/180 45*pi/180];
    Z_nl(1,:) = zeros(5,1)';
    
for i = 1:length(T)-1
        delta_eq(i)=delta_fun(T(i));
        delta_nl(i)=K{i}*(Z_eq(:,i)-Z_nl(i,:)')+delta_eq(i);
        if delta_nl>max(delta_lim)
            delta_nl=max(delta_lim)
        end
        if delta_nl<min(delta_lim)
            delta_nl=min(delta_lim)
        end
        al_f_nl=delta_nl(i)-atan((Z_nl(i,4)+a*Z_nl(i,5))/vx);
        al_r_nl=-atan((Z_nl(i,4)-b*Z_nl(i,5))/vx);
        F_yf_pjk=(b/L)*m*g*D*sin(C*atan(B*(1-E)*al_f_nl+E*atan(B*al_f_nl)));
        F_yr_pjk=(a/L)*m*g*D*sin(C*atan(B*(1-E)*al_r_nl+E*atan(B*al_r_nl)));
        A_Z=[vx*cos(Z_nl(i,3))-Z_nl(i,4)*sin(Z_nl(i,3));
            Z_nl(i,4)*cos(Z_nl(i,3))+vx*sin(Z_nl(i,3));
            Z_nl(i,5);
            (1/m)*(F_yr_pjk+F_yf_pjk*cos(delta_nl(i))-m*vx*Z_nl(i,5));
            (1/Iz)*(-b*F_yr_pjk+a*F_yf_pjk*cos(delta_nl(i)))];
        Z_nl(i+1,:) = Z_nl(i,:) + (T(i+1) - T(i))*A_Z';
    end
Z_nl=Z_nl';
figure;
hold on
plot(Z_eq(1,:),Z_eq(2,:),'--')
plot(Z_nl(1,:),Z_nl(2,:),'-')

max_distance_error=max(sqrt((Z_eq(1,:)-Z_nl(1,:)).^2+(Z_eq(2,:)-Z_nl(2,:)).^2))
% 
