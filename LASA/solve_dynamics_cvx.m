function [W,OptTime] = solve_dynamics_cvx(NonLin,xtau,Data,n,h,V,params)
%This function solves the optimization problem that learns the ELM
%parameters subject to the probabilistic CBF and CLF constraints. 

%The function requires the CVX package to be installed. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
outDim = 2; 

% Compute cost
Nonlin =feval(NonLin,Data(1,:),Data(2,:),Data(3,:),Data(4,:));
x_dot = reshape(Data(5:6,:),[size(Data,2)*2,1]);

% Setup Optimization Problem
fprintf('Solving Dynamics Optimization with CVX... \n');

tic
cvx_begin 
    variables W(n,outDim);
   
    f = reshape(W'*Nonlin,[size(Data,2)*outDim,1]);
    err = f-x_dot;

    %Cost Fucntion
    
    minimize(err'*err);
    
    % Compute Barrier Constraints for all points
    subject to 
        for i=1:size(xtau,2)
           
            x = xtau(i,1);
            y = xtau(i,2);
            z = x-params.xstar(1);
            d = y-params.xstar(2);
          
            h_eval = feval(h,x,y);
            V_eval = feval(V,x,y);
            
            %Computing Barrier 
           gtau = feval(NonLin,x,y,z,d);
% 
%            -1+(W'*gtau-params.xbar)'*params.A*(W'*gtau-params.xbar)+(1-params.gamma)*h_eval <=0 %Barrier Original
% % 
%            (W'*gtau-params.xstar)'*(W'*gtau-params.xstar) -  V_eval <=0 %Lyapunov Original
%        
          
            sqrtvar_CB = norm([2*params.sigma*(W'*gtau-params.xbar)'*params.A, sqrt(2)*params.sigma^2*sqrt(trace(params.A^2))],2) ;
            params.zeta -1+pow_pos(norm([(W'*gtau-params.xbar)'*sqrt(params.A),params.sigma*sqrt(trace(params.A))],2),2)+(1-params.gamma)*h_eval+params.cpk*sqrtvar_CB<=0
% 
            sqrtvar_CL = norm([2*params.sigma*(W'*gtau-params.xstar)'*params.P, sqrt(2)*params.sigma^2*sqrt(trace(params.P^2))],2) ;
            -params.delta+pow_pos(norm([(W'*gtau-params.xstar)'*sqrt(params.P), params.sigma*sqrt(trace(params.P))],2),2)-(1-params.rho)*V_eval+params.cpk*sqrtvar_CL<=0

        end

       
    
cvx_end
OptTime = toc;

end
