%This script learns point-2-point human handwriting motion dynamics using
%ELM approximation whose parameters are learned subject to CBF and CLF
%constraints. The dataset contains of shapes drawn on a Tablet-PC and it is
%publically available. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc; 

%% Preamble

[demos,Data, Inside,boundary,center, theta,mjr_axis, mnr_axis,D,Out,xstar,tau] = data_prep_lasa('ellipse'); %LASA Dataset
inpDim = 4;
outDim = 2;
ActFunc = 'logsig';

%% Defining the Barrier Function 

syms q1 q2 

A = [(cos(theta)^2)/(mjr_axis^2)+(sin(theta)^2)/(mnr_axis^2), cos(theta)*sin(theta)*((1/(mjr_axis^2))-(1/(mnr_axis^2)));...
      cos(theta)*sin(theta)*((1/(mjr_axis^2))-(1/(mnr_axis^2))),(cos(theta)^2)/(mnr_axis^2)+(sin(theta)^2)/(mjr_axis^2)];

h = 1 - ([q1;q2]-center')'*A*([q1;q2]-center');
hFun = matlabFunction(h);

%% Defining the Lyapunov Function
P = eye(2);
V = ([q1;q2]-xstar)'*P*([q1;q2]-xstar);
VFun = matlabFunction(V);
%% -------------------------Design Parameters--------------------------------
params.xstar = xstar;
params.xbar = center';
params.tau = tau;
params.A = A; 
params.P = P;

params.sigma = 0.14 ;

params.pk = 0.8;
params.cpk =  sqrt(2)*erfinv(2*params.pk-1);

params.zeta = 0.1;
params.gamma = 0.80;

params.delta = 0.9;
params.rho = 0.78;


nbsamples = 1000;

n = 25; %number of the hidden neurons

%-----------------Test Design and Forward Prop Parameters------------------

demo_ind = 6;


%% Formulate ELM without constraint

%Dimensions are transposed to match with the following equation \dot{x} = W^T*\sigma(diag(a)*U^T*x+b')

[v,U] = BIP(Data(1:4,:),n,ActFunc);
a = v(1,:);
bias = v(2,:);

if strcmp(ActFunc,'tansig')
    NonLin = @(x,y,z,d) tansig(diag(a)*U*[x;y;z;d]+bias'); %tansigmoid basis
elseif strcmp(ActFunc,'logsig')
    NonLin = @(x,y,z,d) logsig(diag(a)*U*[x;y;z;d]+bias'); %Sigmoid basis
else
    NonLin = @(x,y,z,d) dlarray(diag(a)*U*[x;y;z;d]+bias'); %relu basis    
end

[eps_bar,W0_bar,W0,unconstrainedPos] = unconstrained_elm(NonLin, Data,demos,demo_ind,params);
params.Wbar= W0_bar;
%% Sampling 
fprintf('Getting Sampled Data... \n');
xtau = datasample(Out',nbsamples); 

%% Solve Optimization Problem

WOpt=solve_dynamics_cvx(NonLin,xtau,Data,n,hFun,VFun,params);

%% Temporary Test 

params.NEWxstar = params.xstar;%+.1*randn(outDim,1);

if demo_ind == 1
    estimatePos = Data(1:2,1)+0.2*rand(2,1);%[estimatePos(1,1,m),estimatePos(2,1,m)] = rnd_initialPoints(Data(1:2,1),1,1)%
    Reference = Data(1:2,1:999);
else
    Reference = demos{1,demo_ind}.pos(:,1:end-1);
    estimatePos = demos{1,demo_ind}.pos(:,1)+0.2*rand(2,1);
end

nbPoints = 1; %Number of initial points
for m = 1:nbPoints
     
    for j=1:length(Reference) 

        NonLinDemo_repro(:,j,m) = feval(NonLin,Reference(1,j),Reference(2,j),...
                                                Reference(1,j)-params.xstar(1),Reference(2,j)-params.xstar(2));%,
        estimatePos(:,j+1,m) = WOpt'*NonLinDemo_repro(:,j);    
    end
end
plotresults
