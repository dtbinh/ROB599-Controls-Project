function [AFun,BFun]=linearized_mats_traj(x_ref,u_ref)
syms x u y v psvar r delta_f F_x

load('Linearized_expressions.mat','A_lin','B_lin');
for i=1:length(x_ref)
AFun{i}=double(eval(subs(A_lin,[x u y v psvar r delta_f F_x],[x_ref(:,i)' u_ref(:,i)'])));
BFun{i}=double(eval(subs(B_lin,[x u y v psvar r delta_f F_x],[x_ref(:,i)' u_ref(:,i)'])));
end