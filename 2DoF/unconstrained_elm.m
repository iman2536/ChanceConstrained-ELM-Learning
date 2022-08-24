function [eps_bar,W0_bar,W0,estimatePos] = unconstrained_elm(NonLin, Data,demos,demo_ind,params)
% This function learns the ELM parameters using least square method

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = demos{1,demo_ind}.t;


%Computing Nonlin constraints at demo points 
NonLinDemo = feval(NonLin,Data(1,:),Data(2,:),Data(3,:),Data(4,:));
% NonLinDemo = extractdata(relu(dlX));

% NonLinDemo = feval(NonLin,Data(1,:),Data(2,:),Data(3,:),Data(4,:));%
T = [Data(5,:);Data(6,:)];
W0 = pinv(NonLinDemo') * T';

if demo_ind == 1
    estimatePos = Data(1:2,1)+0.2*rand(2,1);
    Reference = Data(1:2,1:999);
else
    Reference = demos{1,demo_ind}.q(:,1:end-1);
    estimatePos = demos{1,demo_ind}.q(:,1)+0.2*rand(2,1);
end


% The math is for one-step prediction! 
for j=1:length(time)-2 
    NonLinDemo_repro(:,j) =feval(NonLin,estimatePos(1,j),estimatePos(2,j),estimatePos(1,j)-params.xstar(1),estimatePos(2,j)-params.xstar(2));%
    estimatePos(:,j+1) = W0'*NonLinDemo_repro(:,j);

end



Test = estimatePos(1:2,:);
Error = Reference - Test;

for i=1:size(Error,2)
        L2_Error(i) = norm(Error(:,i),2);
end


%Returning output
eps_bar = norm(L2_Error,'inf');

W0_bar = norm(W0, 'fro');

end


