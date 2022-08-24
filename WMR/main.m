%This is the main script that learns the controlled dynamics of a wheeled
%mobile robots using ELM approximation with probabilistic control barrier
%and Lyapunov functions 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc; 

%% Preamble
[demos,Data,Inside,Out, center, theta, axes,tau] = dataprep_wmr();
inpDim = 5;
outDim = 3;
ActFunc = 'logsig';
demo_ind = 2;
%% Defining the Barrier Function 

syms x y z
A = [1/axes(1)^2,0,0;0,1/axes(2)^2,0;0,0,1/axes(3)^2];

h = 1-([x;y;z]-center')'*A*([x;y;z]-center');
hFun = matlabFunction                                                                                                                                                                                                                                                                   (h);


%% -------------------------Design Parameters--------------------------------
params.xstar = demos{1,demo_ind}.Xstar;
params.xbar = center';
params.tau = tau;
params.A = A; 
params.sigma = 0.02 ;

params.pk = 0.8;
params.cpk =  sqrt(2)*erfinv(2*params.pk-1);

params.zeta = 0.5;
params.gamma = 0.9;

params.delta = 1;
params.rho = 0.9;

nbsamples = 1000;

n = 25; %number of the hidden neurons

%-----------------Test Design and Forward Prop Parameters------------------
nb_MCrun = 10;

radius = 0.1;

%% Formulate ELM without constraint

%Dimensions are transposed to match with the following equation \dot{x} = W^T*\sigma(diag(a)*U^T*x+b')

[v,U] = BIP(Data(1:inpDim,:),n,ActFunc);
a = v(1,:);
bias = v(2,:);

if strcmp(ActFunc,'tansig')
    NonLin = @(x,y,z,d,e) tansig(diag(a)*U*[x;y;z;d;e]+bias'); %tansigmoid basis
elseif strcmp(ActFunc,'logsig')
    NonLin = @(x,y,z,d,e) logsig(diag(a)*U*[x;y;z;d;e]+bias'); %Sigmoid basis
else
    NonLin = @(x,y,z,d,e) dlarray(diag(a)*U*[x;y;z;d;e]+bias'); %relu basis    
end


params.gbar = (norm(diag(a),"fro")*norm(U,"fro")*sqrt(n))/(2*sqrt(2));
[eps_bar,W0_bar,W0,unconstrainedPos] = unconstrained_elm(NonLin, Data,demos,demo_ind,params);
params.Wbar= W0_bar;

%% Defining the Lyapunov Function
% Learning Lyapunov params
P = eye(3);
V = ([x;y;z]-params.xstar)'*P*([x;y;z]-params.xstar);
VFun = matlabFunction(V);

params.P = P;
%% Sampling 
fprintf('Getting Sampled Data... \n');
xtau = datasample(Out',nbsamples); 

%% Solve Optimization Problem

WOpt=solve_dynamics_cvx(NonLin,xtau',Data,n,hFun,VFun,params);

%% Temporary Test 

params.NEWxstar = params.xstar;%+0.05*randn(outDim,1);

if demo_ind == 1
    estimatePos = Data(1:3,1);%+0.5*rand(2,1);
    Reference = Data(1:3,1:999);
else
    Reference = Data(1:3,(demo_ind-1)*999+1:(demo_ind-1)*999+999);
    estimatePos = Data(1:3,(demo_ind-1)*999+1);%+0.5*rand(3,1);
end

nbPoints = 1; % Number of initial points
for m = 1:nbPoints
     
    for j=1:length(Reference) 
        
        NonLinDemo_repro(:,j,m) = feval(NonLin,Reference(1,j),Reference(2,j),Reference(3,j),...
                                        Reference(1,j)-params.NEWxstar(1),Reference(2,j)-params.NEWxstar(2));%
        estimatePos(:,j+1,m) = WOpt'*NonLinDemo_repro(:,j);
        
    end
end

time = 0:demos{1,1}.dt:999;

plotresults

