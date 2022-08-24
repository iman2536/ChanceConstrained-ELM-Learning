function [demos,Data, Inside,boundary,center, theta,mjr_axis, mnr_axis,D,Out,xstar,tau] = data_prep_lasa(invRegion)
%This function prepares LASA data set to be used to learn the end-effector
% dynamics subject to Barrier constraints 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

names = {'Angle','BendedLine','CShape','DoubleBendedLine','GShape',...
         'heee','JShape','JShape_2','Khamesh','Leaf_1',...
         'Leaf_2','Line','LShape','NShape','PShape',...
         'RShape','Saeghe','Sharpc','Sine','Snake',...
         'Spoon','Sshape','Trapezoid','Worm','WShape','Zshape',...
         'Multi_Models_1','Multi_Models_2','Multi_Models_3','Multi_Models_4'};

% figName = [];
n =2 ;
%% preprocessing
load_dir = 'DataSet/';
DemoName = names{n};
load([load_dir,DemoName,'.mat'],'demos','dt');  %loading the model



%%% plotting the result
% if isempty(figName) || ~ishandle(figName)
%     figName = figure('name',DemoName,'position',[100   100   600   800]);
% end
% sp(1) = subplot(1,1,1);
% hold on; box on; grid on
trans = [20;10];

demo_ind = 7;
% % 
for i = 1:demo_ind
    demos{1,i}.pos=demos{1,i}.pos+trans;
%     plot(demos{1,i}.pos(1,:),demos{1,i}.pos(2,:))
%     hold on
end

% xlabel(sp(1),'x (mm)','fontsize',15);
% ylabel(sp(1),'y (mm)','fontsize',15);
% title(sp(1),names{n},'fontsize',15)

%% Defining the invariant region
tau = .1;
center = [-25,5]+trans';
if strcmp(invRegion, 'ellipse')
    mjr_axis = 30; mjr_axis_2 = mjr_axis+5;
    
    mnr_axis = 26; mnr_axis_2 = mnr_axis+5;
    theta = pi/6;
    x1 = -50:tau:40; y1 = -30:tau:60;
else 
    mjr_axis = 27.48;
    mnr_axis = 27.48;
    theta = pi/2;
    x1 = -52:tau:38; y1 = -55:tau:35;
end 

% plotellipse(center, mjr_axis, mnr_axis, theta,[0.6,0.2,0],3);
% hold on
% plotellipse(center, mjr_axis_2, mnr_axis_2, theta,[0.2,0.1,0],1);
% hold on;
%% Finding points inside the ellipse 

%%% Create a meshgrid to include all the demonstrations
[X1,Y1]=meshgrid(x1,y1);
%%% Drawing points from inside the ellipse 
InsideX1 = []; OutX1 = [];
InsideY1 = []; OutY1 = [];

A1 = [(cos(theta)^2)/(mjr_axis^2)+(sin(theta)^2)/(mnr_axis^2), cos(theta)*sin(theta)*((1/(mjr_axis^2))-(1/(mnr_axis^2)));...
      cos(theta)*sin(theta)*((1/(mjr_axis^2))-(1/(mnr_axis^2))),(cos(theta)^2)/(mnr_axis^2)+(sin(theta)^2)/(mjr_axis^2)];

A2 = [(cos(theta)^2)/(mjr_axis_2^2)+(sin(theta)^2)/(mnr_axis_2^2), cos(theta)*sin(theta)*((1/mjr_axis_2^2)-(1/mnr_axis_2^2));...
      cos(theta)*sin(theta)*((1/mjr_axis_2^2)-(1/mnr_axis_2^2)),(cos(theta)^2)/(mnr_axis_2^2)+(sin(theta)^2)/(mjr_axis_2^2)];


for i = 1:numel(X1)
    if ([X1(i);Y1(i)]-center')'*A1*([X1(i);Y1(i)]-center')-1<=0
        InsideX1 = [InsideX1,X1(i)];
        InsideY1 = [InsideY1,Y1(i)];
    end
    % Drawing points that are close to the boundary of the ellipse
    if ([X1(i);Y1(i)]-center')'*A2*([X1(i);Y1(i)]-center')-1<=0      
        OutX1 = [OutX1,X1(i)];
        OutY1 = [OutY1,Y1(i)];
    end
end
% hold on;plot(InsideX1,InsideY1,'ok','MarkerSize',2);
Inside = [InsideX1;InsideY1];
Out = [OutX1;OutY1];
boundary = setdiff(Inside',Out','rows');
boundary = boundary';

%Points on Demostrations
InsideOnDemo_pos = [];
InsideOnDemo_vel = []; 



for i = 1:demo_ind
    %Position
    tmp = demos{1,i}.pos(:,1:end-1);  
    InsideOnDemo_pos = [InsideOnDemo_pos,tmp];
    %velocity
    tmp = demos{1,i}.pos(:,2:end);
    InsideOnDemo_vel = [InsideOnDemo_vel,tmp];
 
end
xstar = trans;
err = InsideOnDemo_pos - xstar;

Data = [InsideOnDemo_pos;err;InsideOnDemo_vel];
D = [x1(1) y1(1);x1(end) y1(end)];
end

