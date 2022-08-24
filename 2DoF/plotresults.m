% This script plots the result of the ELM parameter learning of a 2DoF systems with 
% CBF and CLF constraints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plots
nbDemo = 7;
hfig1 = figure(1);
[lg(1),bnds]=plotellipse(center, mjr_axis, mnr_axis, theta,[0.6,0.2,0],3);
hold on
lg(2)=plotellipse(center,mjr_axis+.5, mnr_axis+.5, theta,[0.2,0.1,0],1);%mjr_axis+5, mnr_axis+5,
hold on;

% Demonstrations

for i = 1:nbDemo
    
    if i == 1
        lg(3) = plot(demos{1,i}.q(1,1),demos{1,i}.q(2,1),'ok','MarkerSize',6,'LineWidth',3); hold on
        lg(4)= plot(demos{1,i}.q(1,end),demos{1,i}.q(2,end),'*k','MarkerSize',18,'LineWidth',3); hold on
        lg(5) = plot(demos{1,i}.q(1,:),demos{1,i}.q(2,:),'r-','LineWidth',2); hold on;
    else 
        plot(demos{1,i}.q(1,1),demos{1,i}.q(2,1),'ok','MarkerSize',6,'LineWidth',3); hold on
        plot(demos{1,i}.q(1,:),demos{1,i}.q(2,:),'r-','LineWidth',2); hold on;
    end
end

%Sampled points

lg(6) = plot(xtau(:,1),xtau(:,2),'o','MarkerSize',6,'Color',[0.6,0.6,0.6]);

% Estimated pos
for i =1:nbPoints
    lg(7) = plot(estimatePos(1,:,i),estimatePos(2,:,i),'--','Color',[0.0,0.6,0.2],'LineWidth',4); hold on;
    plot(estimatePos(1,1,i),estimatePos(2,1,i),'ok','MarkerSize',6,'LineWidth',3); hold on;
    plot(estimatePos(1,end,i),estimatePos(2,end,i),'*k','MarkerSize',18,'LineWidth',3); hold on;
end

% Unconstrained Pos
lg(8) = plot(unconstrainedPos(1,:),unconstrainedPos(2,:),'b--','LineWidth',3);
hold on 
plot(unconstrainedPos(1,end),unconstrainedPos(2,end),'*k','MarkerSize',18,'LineWidth',3); hold on
plot(unconstrainedPos(1,1),unconstrainedPos(2,1),'ok','MarkerSize',6,'LineWidth',3);
% Logistics 
xlabel('$x_1 \ [\mathrm{mm}]$','Interpreter','latex')
ylabel('$x_2 \ [\mathrm{mm}]$','Interpreter','latex')


legend([lg(1),lg(5),lg(8),lg(7),lg(3),lg(4),lg(6)],'Invariant Region','Demos',...
'Unconstrained','Constrained','Initial Conditions','Target Location',...
'Sampled Points','Interpreter','latex','NumColumns',1,'Orientation','vertical','Location','northwest','FontSize', 32)

set(gca,'FontSize',36)
% axis square
% axis([-2.5 2.5 -2.5 2.5])

% MagInset(hfig1, -1, [1.5 1.9 -2.1 -1.65], [1.25 2.25 0.8 2.1], {'NW','SW';'NE','SE'});
% set(gca,'FontSize',30)
%% Plot angles vs time

time = linspace(0,10,1000);
load_dir =  '/Users/iman/Documents/UConn Repo/imanSalehi/Current Work/Journal Articles/OJCSYS/Code/demos_2DoF/NewDemo1/';
 FileName=['demo',num2str(demo_ind)];
 load([load_dir,FileName,'.mat'],'th1','th2');  %loading the model

figure(2)
subplot(2,1,1) 

leg(1) = plot(time(1:end),estimatePos(1,:),'--','Color',[0.0,0.6,0.2],'LineWidth',4); hold on
leg(2) = plot(time,th1,'r-','LineWidth',3);
% Logistics 
xlabel('$\mathrm{Time} \ [s]$','Interpreter','latex')
ylabel('$q_1 \ [\mathrm{rad}]$','Interpreter','latex')

% legend([leg(1),leg(2)],'Learned','Actual','Interpreter','latex','FontSize', 30)
set(gca,'FontSize',30)

leg(3) = plot(time(1:end-1),unconstrainedPos(1,:),'--','Color',[0,0,1],'LineWidth',3); hold on


leg(4)=plot(time,max(bnds(1,:))*ones(1,length(time)),'k--','LineWidth',2); hold on
leg(5)=plot(time,min(bnds(1,:))*ones(1,length(time)),'k--','LineWidth',2); hold on

% plot(bnds(1,:),'k--','LineWidth',2)
xlabel('$\mathrm{Time} \ [s]$','Interpreter','latex')
ylabel('$q_1 \ [\mathrm{rad}]$','Interpreter','latex')

legend([leg(2),leg(1),leg(3),leg(4)],'Actual','Constrained','Unconstrained','bounds','Interpreter','latex','FontSize', 30)
set(gca,'FontSize',32)

MagInset(figure(2), -1, [0.6 2 3.2 3.8], [2 4 1.8 2.8], {'SW','SW';'NE','NE'}); hold on
set(gca,'FontSize',30)
subplot(2,1,2) 

leg(6) = plot(time(1:end),estimatePos(2,:),'--','Color',[0.0,0.6,0.2],'LineWidth',3); hold on
leg(7) = plot(time,th2,'r-','LineWidth',3);
leg(8) = plot(time(1:end-1),unconstrainedPos(2,:),'--','Color',[0,0,1],'LineWidth',3); hold on

leg(9)=plot(time,max(bnds(2,:))*ones(1,length(time)),'k--','LineWidth',2); hold on
leg(10)=plot(time,min(bnds(2,:))*ones(1,length(time)),'k--','LineWidth',2); hold on

xlabel('$\mathrm{Time} \ [s]$','Interpreter','latex')
ylabel('$q_2 \ [\mathrm{rad}]$','Interpreter','latex')
legend([leg(7),leg(6),leg(8),leg(9)],'Actual','Constrained','Unconstrained','bounds','Interpreter','latex','FontSize', 30)
set(gca,'FontSize',32)
% MagInset(figure(2), -1, [2.16 2.64 1.71 1.91], [4 5 2.1 2.9], {'NW','NW';'SE','SE'}); hold on
% MagInset(figure(2), -1, [1.7 2.8 -2 -1.7 ], [3.5 5.6 -.7 -0.1], {'NW','NW';'SE','SE'});
set(gca,'FontSize',30)