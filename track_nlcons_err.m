function [c,ceq,J,Jeq] = track_nlcons_err(x,x_ref,TestTrack,Ndec,PredHorizon,n,m)

%Vecor to CM of car must lie inside track vector
c=zeros(2*(PredHorizon+1),1);% Compute nonlinear inequalities at x.
J=zeros(Ndec,2*(PredHorizon+1));
for kk=1:(PredHorizon+1)
     xv=x(n*(kk-1)+1)+x_ref(n*(kk-1)+1); yv=x(n*(kk-1)+3)+x_ref(n*(kk-1)+3);
    %Find closest points on track
    [mindist,ind]=sort(((xv-TestTrack.cline(1,:)).^2+(yv-TestTrack.cline(2,:)).^2),'ascend');
    %Locations of closest centerline points
    xc1=TestTrack.cline(1,ind(1)); yc1=TestTrack.cline(2,ind(1));
    xc2=TestTrack.cline(1,ind(2)); yc2=TestTrack.cline(2,ind(2));
    mc=(yc2-yc1)/(xc2-xc1);%Slope of centerline segment
    %Locations of closest left track points
    xl1=TestTrack.bl(1,ind(1)); yl1=TestTrack.bl(2,ind(1));
    xl2=TestTrack.bl(1,ind(2)); yl2=TestTrack.bl(2,ind(2));
    ml=(yl2-yl1)/(xl2-xl1);%Slope of left track segment
    cl=yl2-ml*xl2;%Yaxis intersection for left track segment
    %Locations of closest right track points
    xr1=TestTrack.br(1,ind(1)); yr1=TestTrack.br(2,ind(1));
    xr2=TestTrack.br(1,ind(2)); yr2=TestTrack.br(2,ind(2));
    mr=(yr2-yr1)/(xr2-xr1);%Slope of right track segment
    cr=yr2-mr*xr2;%Yaxis intersection for right track segment
    
    %Location of point at closest perpendicular distance from vehicle on
    %centerline
    xc=xc1+(1/(1+mc^2))*((xv-xc1)+mc*(yv-yc1));
    yc=yc1+(mc/(1+mc^2))*((xv-xc1)+mc*(yv-yc1));
    
    %Intermediate variables (invbl and invbr should be both < 1  for car to lie inside
    %track). Represents ratio of perpendicular distances from vehicle and track left and right to centerline)  
    invbr=((yv-yc)-mr*(xv-xc))/(mr*xc+cr-yc);
    invbl=((yv-yc)-ml*(xv-xc))/(ml*xc+cl-yc);
    
    c(2*kk-1,1)=invbr-0.95;
    c(2*kk,1)=invbl-0.95;
    
    dxc_by_xv=(1/(1+mc^2));
    dxc_by_yv=(mc/(1+mc^2));
    dyc_by_xv=(mc/(1+mc^2));
    dyc_by_yv=(mc^2/(1+mc^2));
    
    dinvbr_by_xv=-mr/(mr*xc+cr-yc);
    dinvbr_by_yv=1/(mr*xc+cr-yc);
    dinvbr_by_xc=mr/(mr*xc+cr-yc)-((mr^2*xc)/(mr*xc+cr-yc)^2);
    dinvbr_by_yc=-1/(mr*xc+cr-yc)-((yc)/(mr*xc+cr-yc)^2);
    
    dinvbl_by_xv=-ml/(ml*xc+cl-yc);
    dinvbl_by_yv=1/(ml*xc+cl-yc);
    dinvbl_by_xc=ml/(ml*xc+cl-yc)-((ml^2*xc)/(ml*xc+cl-yc)^2);
    dinvbl_by_yc=-1/(ml*xc+cl-yc)-((yc)/(ml*xc+cl-yc)^2);
    
    J(n*(kk-1)+1,2*kk-1)=(dinvbr_by_xv+dinvbr_by_xc*dxc_by_xv+dinvbr_by_yc*dyc_by_xv);%Derivarive of c(2*kk-1,1) wrt xv
    J(n*(kk-1)+3,2*kk-1)=(dinvbr_by_yv+dinvbr_by_xc*dxc_by_yv+dinvbr_by_yc*dyc_by_yv);%Derivarive of c(2*kk-1,1) wrt yv
    J(n*(kk-1)+1,2*kk)=(dinvbl_by_xv+dinvbl_by_xc*dxc_by_xv+dinvbl_by_yc*dyc_by_xv);%Derivarive of c(2*kk,1) wrt xv
    J(n*(kk-1)+3,2*kk)=(dinvbl_by_yv+dinvbl_by_xc*dxc_by_yv+dinvbl_by_yc*dyc_by_yv);%Derivarive of c(2*kk,1) wrt yv
end

ceq = [];   % Compute nonlinear equalities at x.
Jeq=[];