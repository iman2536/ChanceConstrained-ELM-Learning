function [demos,Data,Inside,Out, center, theta, axes,tau] = dataprep_wmr()
%This function preprocesses the demonstration data and define the invariant 
%region
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Drawing Ellipse
close all; clear all;clc

demos = [];
nbDemo = 5; 
load_dir = 'DataSet/';%'C:\Users\ims16102\Documents\UConn Repo\imanSalehi\Current Work\Journal Articles\TBD\Code\WMR\DataGen\demos_WMR\';%;%;%
for i = 1: nbDemo
   
    FileName=['demo',num2str(i)];
    load([load_dir,FileName,'.mat'],'vref','X','dt');  %loading the model
    
    %Saving data
    demos{1,i}.X = X;
    demos{1,i}.Xstar = X(:,end);
    demos{1,i}.u = vref;
    demos{1,i}.dt = dt;
end



%% Plotting the demos
% for i = 1:nbDemo
%     
%     plot3(demos{1,i}.X(1,:),demos{1,i}.X(2,:),demos{1,i}.X(3,:))
%     hold on
%     plot3(demos{1,i}.X(1,end),demos{1,i}.X(2,end),demos{1,i}.X(3,end),'*k')
%     hold on
%     plot3(demos{1,i}.X(1,1),demos{1,i}.X(2,1),demos{1,i}.X(3,1),'ok')
% end

% for i = 1:nbDemo
%     plot(demos{1,i}.X(3,:))
%     hold on
% end
% hold on
% xlabel('x')
% ylabel('y')
% zlabel('z')
%% Defining the invariant region
theta = -pi/4;
center = [-0.5, 0.51,1.38];
tau = 0.1;

a = 2.5;b = 2;c=3.4; % Ellipsoid semi-axes
axes = [a,b,c];
% plotellipsoid(center,a,b,c,[0.2,0.2,0],.5);

a2 = a+.5;
b2 = b+0.5;
c2 = c+0.5;

% hold on
% plotellipsoid(center,a2,b2,c2,[0.83,0.82,0.78],0.5);

%% Finding points inside the ellipse 
%%% Create a meshgrid to include all the demonstrations
x1 = -5:tau:4;
x2 = -4:tau:5;
x3 = -5:tau:8;
% x1 = -2:tau:1.07;
% x2 = -0.59:tau:1.613;
% x3 = -0.723:tau:3.513;
[X1,Y1,Z1] = meshgrid(x1,x2,x3);

%%% Drawing points from inside the ellipse 
InsideX1 = []; OutX1 = []; 
InsideY1 = []; OutY1 = []; 
InsideZ1 = []; OutZ1 = []; 

for i = 1:numel(X1)
    if  ((X1(i)-center(1))^2)/(a^2)+((Y1(i)-center(2))^2)/(b^2)+((Z1(i)-center(3))^2)/(c^2) -1 <= 0
        InsideX1 = [InsideX1,X1(i)];
        InsideY1 = [InsideY1,Y1(i)];
        InsideZ1 = [InsideZ1,Z1(i)];
    end
        % Drawing points from inside the second ellipse
    if ((X1(i)-center(1))^2)/(a2^2)+((Y1(i)-center(2))^2)/(b2^2)+((Z1(i)-center(3))^2)/(c2^2) -1 <= 0
        
        OutX1 = [OutX1,X1(i)];
        OutY1 = [OutY1,Y1(i)];
        OutZ1 = [OutZ1,Z1(i)];
        
    end
 
end

Inside = [InsideX1;InsideY1;InsideZ1];
Out = [OutX1;OutY1;OutZ1];

%Points on Demostrations

InsideOnDemo_pos = []; 
InsideOnDemo_vel = []; 

for i = 1:nbDemo

    %Position
    tmp = demos{1,i}.X(:,1:end-1);  
    InsideOnDemo_pos = [InsideOnDemo_pos,tmp];
    %velocity
    tmp = demos{1,i}.X(:,2:end);
    InsideOnDemo_vel = [InsideOnDemo_vel,tmp];
    
end

err = InsideOnDemo_pos(1:2,:) - demos{1,1}.Xstar(1:2,1); 
Data  = [InsideOnDemo_pos;err;InsideOnDemo_vel];

% hold on
% plot3(OutX1,OutY1,OutZ1,'o','MarkerSize',3);

