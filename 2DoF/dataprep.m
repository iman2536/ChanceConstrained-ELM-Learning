function [demos,Data,Inside,Out, center, theta,mjr_axis, mnr_axis,D,xstar,tau] = dataprep()
%This function preprocesses the demonstration data and define the invariant 
%region
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Drawing Ellipse
close all; clear all;clc

demos = [];
nbDemo = 7;
load_dir =  'DataSet/';
for i = 1: nbDemo
   
    FileName=['demo',num2str(i)];
    load([load_dir,FileName,'.mat'],'th1','th2','tau1','tau2','Ts');  %loading the model
    tmp1 = tau1(end,1);
    tmp2 = tau1(end,1);
    tau1(1000,1) = tmp1;
    tau2(1000,1) = tmp2;
    %Saving data
    demos{1,i}.q = [th1';th2'];
    demos{1,i}.u = [tau1';tau2'];
    demos{1,i}.t = Ts;
    demos{1,i}.dt = mode(diff(Ts));
end

%% Defining the invariant region
mjr_axis = 1.3;
mnr_axis = 1.2;

theta = -pi/12;
center = [2.3, -0.8];
tau = 0.01;

q1 = -1.5:tau:4.5;
q2 = -2.5:tau:1;

% plotellipse(center, mjr_axis, mnr_axis, theta,[0.6,0.2,0],3);
% 
mjr_axis_2 = mjr_axis +0.5;
mnr_axis_2 = mnr_axis + 0.5; 
% 
% hold on
% plotellipse(center, mjr_axis_2, mnr_axis_2, theta,[0.2,0.1,0],1);
% hold on;
%% Plotting the demos

% for i = 1:nbDemo
%     hold on
%     plot(demos{1,i}.q(1,:),demos{1,i}.q(2,:),'r-','LineWidth',2)
%     plot(demos{1,i}.q(1,1),demos{1,i}.q(2,1),'ok','MarkerSize',6,'LineWidth',3)
%     if i == 1
%         plot(demos{1,i}.q(1,end),demos{1,i}.q(2,end),'*k','MarkerSize',16,'LineWidth',3)
%     end
% end
xstar = demos{1,1}.q(:,end);
%% Finding points inside the ellipse 

%%% Create a meshgrid to include all the demonstrations
[Q1,Q2]=meshgrid(q1,q2);

%%% Drawing points from inside the ellipse 
InsideQ1 = []; OutQ1 = []; 
InsideQ2 = []; OutQ2 = []; 


for i = 1:numel(Q1)
    if (((Q1(i)-center(1))*cos(theta)+(Q2(i)-center(2))*sin(theta))^2)/(mjr_axis^2)+(((Q1(i)-center(1))*sin(theta)-(Q2(i)-center(2))*cos(theta))^2)/(mnr_axis^2)-1<=0
        InsideQ1 = [InsideQ1,Q1(i)];
        InsideQ2 = [InsideQ2,Q2(i)];

    end
        % Drawing points that are close to the boundary of the ellipse
    if (((Q1(i)-center(1))*cos(theta)+(Q2(i)-center(2))*sin(theta))^2)/(mjr_axis_2^2)+(((Q1(i)-center(1))*sin(theta)-(Q2(i)-center(2))*cos(theta))^2)/(mnr_axis_2^2)-1<=0        
        OutQ1 = [OutQ1,Q1(i)];
        OutQ2 = [OutQ2,Q2(i)];
    end
end

Inside = [InsideQ1;InsideQ2];
Out = [OutQ1;OutQ2];

boundary = setdiff(Inside',Out','rows');
boundary = boundary';


%Points on Demostrations
InsideOnDemoQ1 = []; uDemo1 = []; VelDemoX = []; 
InsideOnDemoQ2 = []; uDemo2 = []; VelDemoY = [];

for i = 1:nbDemo
    
    %Position - k instances 
    tmp = demos{1,i}.q(:,1:end-1);  
    InsideOnDemoQ1 = [InsideOnDemoQ1,tmp];

    tmp = demos{1,i}.q(:,2:end);
    InsideOnDemoQ2 = [InsideOnDemoQ2,tmp];
    
end


err = InsideOnDemoQ1 - xstar;

Data = [InsideOnDemoQ1;err;InsideOnDemoQ2];
D = [q1(1) q2(1);q1(end) q2(end)];

% plot(Q1',Q2','oy','MarkerSize',3);
% plot(InsideQ1,InsideQ2,'ok','MarkerSize',2);