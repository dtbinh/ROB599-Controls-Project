function [cost,J]=error_cost(var,H,f)
cost=0.5*var'*H*var+f'*var;
J=H*var+f;