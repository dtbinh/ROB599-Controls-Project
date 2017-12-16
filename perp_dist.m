function pdist=perp_dist(xv,yv,cline) 
    %Find closest points on track centerline
    [mindist,ind]=sort(((xv-cline(1,:)).^2+(yv-cline(2,:)).^2),'ascend');
    
    %Locations of closest centerline points
    xc1=cline(1,ind(1)); yc1=cline(2,ind(1));
    xc2=cline(1,ind(2)); yc2=cline(2,ind(2));
    mc=(yc2-yc1)/(xc2-xc1);%Slope of centerline segment
    
    %Location of point at closest perpendicular distance from xv,yv on
    %centerline
    xc=xc1+(1/(1+mc^2))*((xv-xc1)+mc*(yv-yc1));
    yc=yc1+(mc/(1+mc^2))*((xv-xc1)+mc*(yv-yc1));
    
    pdist=sqrt((xc-xv)^2+(yv-yc)^2);