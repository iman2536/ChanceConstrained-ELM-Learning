function [W,OptTime] = solve_dynamics_cvx(NonLin,xtau,Data,n,h,V,params)
%This function solves the optimization problem that learns the ELM
%parameters subject to the probabilistic CBF and CLF constraints. 

%The function requires the CVX package to be installed. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

outDim = 3; 

% Compute cost
Nonlin =feval(NonLin,Data(1,:),Data(2,:),Data(3,:),Data(4,:),Data(5,:));%,Data(3,:)-params.xstar(1),Data(4,:)-params.xstar(2)
% Nonlin = extractdata(relu(dlX));
x_dot = reshape(Data(6:8,:),[size(Data,2)*outDim,1]);%Data(5:6,:)


% Setup Optimization Problem


fprintf('Solving Dynamics Optimization with CVX... \n');

tic
cvx_begin 
    variable W(n,outDim);
    
    f = reshape(W'*Nonlin,[size(Data,2)*outDim,1]);
    err = f-x_dot;

    
    %Cost Fucntion
    minimize(err'*err);
    
    % Compute Barrier Constraints for all points
    subject to 
        for i=1:size(xtau,1)
%            
            x = xtau(1,i);
            y = xtau(2,i);
            z = xtau(3,i);
            d = x-params.xstar(1);
            e = y-params.xstar(2);
%             f = z-params.xstar(3);
            
            h_eval = feval(h,x,y,z);
            V_eval = feval(V,x,y,z);
            
            gtau = feval(NonLin,x,y,z,d,e);
%             gtau = extractdata(relu(dlX1));
            % Computing Barrier 

%            -1+(W'*gtau-params.xbar)'*params.A*(W'*gtau-params.xbar)+(1-params.gamma)*h_eval <=0
%       
            sqrtvar_CB = norm([2*params.sigma*(W'*gtau-params.xbar)'*params.A, sqrt(2)*params.sigma^2*sqrt(trace(params.A^2))],2) ;
            params.zeta -1+pow_pos(norm([(W'*gtau-params.xbar)'*sqrt(params.A),params.sigma*sqrt(trace(params.A))],2),2)+(1-params.gamma)*h_eval+params.cpk*sqrtvar_CB<=0
% 
            sqrtvar_CL = norm([2*params.sigma*(W'*gtau-params.xstar)'*params.P, sqrt(2)*params.sigma^2*sqrt(trace(params.P^2))],2) ;
            -params.delta+pow_pos(norm([(W'*gtau-params.xstar)'*sqrt(params.P),params.sigma*sqrt(trace(params.P))],2),2)-(1-params.rho)*V_eval+params.cpk*sqrtvar_CL<=0
%             (W'*gtau-params.xstar)'*(W'*gtau-params.xstar) -  V_eval <=0

 
        end
    
cvx_end
OptTime = toc;

end
