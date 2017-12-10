clear;clc;
load('TestTrack.mat')

bl_x = TestTrack.bl(1,:);
bl_y = TestTrack.bl(2,:);

br_x = TestTrack.br(1,:);
br_y = TestTrack.br(2,:);

cline_x = TestTrack.cline(1,:);
cline_y = TestTrack.cline(2,:);

theta = TestTrack.theta(1,:);

U1 = [-0.004*ones(500,1) 5000*ones(500,1)];
U2 = [-0.5*ones(250,1) -7000*ones(250,1);];
U3 = [-0.5*ones(400,1) 3450*ones(400,1);];
U4 = [-0.05*ones(170,1) 5000*ones(170,1)];
U5 = [0.017*ones(350,1) 1000*ones(350,1)];
U6 = [0.5*ones(300,1) -7100*ones(300,1)];
U7 = [-0.008*ones(300,1) 2000*ones(300,1)];
U8 = [-0.5*ones(425,1) 200*ones(425,1)];
U9 = [0.5*ones(400,1) 300*ones(400,1)];
U10 = [0.01*ones(200,1) 5000*ones(200,1)];
U11 = [-0.5*ones(330,1) 1000*ones(330,1)];
U12 = [-0.5*ones(400,1) -400*ones(400,1)];
U13 = [-0.5*ones(400,1) 1000*ones(400,1)];
U14 = [-0.2*ones(110,1) 5000*ones(110,1)];
U15 = [0.5*ones(120,1) -4800*ones(120,1)];
U16 = [0.5*ones(500,1) 1000*ones(500,1)];
U17 = [0.02*ones(250,1) 5000*ones(250,1)];
U18 = [-0.5*ones(250,1) -5700*ones(250,1)];
U19 = [-0.5*ones(700,1) 1800*ones(700,1)];
U20 = [0.5*ones(240,1) -3790*ones(240,1)];
U21 = [0.5*ones(50,1) 5000*ones(50,1)];
U22 = [0.07*ones(103,1) 5000*ones(103,1)];
U23 = [-0.0005*ones(430,1) 5000*ones(430,1)];
U24 = [0.5*ones(280,1) -9000*ones(280,1)];
U25 = [0.012*ones(500,1) 5000*ones(500,1)];
U26=[-0.04*ones(40,1) 5000*ones(40,1)];
U27=[0*ones(460,1) 5000*ones(460,1)];


U = [U1;U2;U3;U4;U5;U6;U7;U8;U9;U10;U11;U12;U13;U14;U15;U16;U17;U18;U19;U20;U21;U22;U23;U24;U25;U26;U27];

Z = forwardIntegrateControlInput(U); 

plot(bl_x,bl_y,br_x,br_y,cline_x,cline_y,'--')
hold on
h = animatedline;
x = Z(:,1);
y = Z(:,3);

for k = 1:length(x)
    addpoints(h,x(k),y(k));
    drawnow limitrate
    pause(0.001)
end
