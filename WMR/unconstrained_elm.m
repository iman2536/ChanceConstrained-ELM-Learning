function [eps_bar,W0_bar,W0,estimatePos] = unconstrained_elm(NonLin, Data,demos,demo_ind,params)
% This function learns the ELM parameters using least square method

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = 0:demos{1,demo_ind}.dt:10;
OutDim = 3;

%Computing Nonlin constraints at demo points 
NonLinDemo = feval(NonLin,Data(1,:),Data(2,:),Data(3,:),Data(4,:),Data(5,:));
% NonLinDemo = extractdata(relu(dlX));


T = Data(6:8,:);%Data(3,:);Data(4,:)
W0 = pinv(NonLinDemo') * T';

if demo_ind == 1
    estimatePos = Data(1:OutDim,1);
    Reference = Data(1:OutDim,1:demo_ind*999);
else
    Reference = Data(1:OutDim,(demo_ind-1)*999+1:demo_ind*999);
    estimatePos = Data(1:OutDim,(demo_ind-1)*999+1);
end
% The math is for one-step prediction! 
for j=1:length(time)-3 
    NonLinDemo_repro(:,j) = feval(NonLin,estimatePos(1,j),estimatePos(2,j),estimatePos(3,j),estimatePos(1,j)-params.xstar(1),estimatePos(2,j)-params.xstar(2));
    estimatePos(:,j+1) = W0'*NonLinDemo_repro(:,j);

end


% hold on 
% plot(estimatePos(1,:),estimatePos(2,:),'b--','LineWidth',3)


Test = estimatePos(1:OutDim,:);
Error = Reference - Test;

for i=1:size(Error,2)
        L2_Error(i) = norm(Error(:,i),2);
end


%Returning output
eps_bar = norm(L2_Error,'inf');

W0_bar = norm(W0, 'fro');

end


