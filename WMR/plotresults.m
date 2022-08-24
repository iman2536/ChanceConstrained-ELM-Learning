%% Plot the results
%This script plots the results of the ELM parameter learning a wheeled
%mobile robots subject to the probabilistic CBF and CLF constraints. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbDemo = 5;

a2 = axes(1)+0.5;
b2 = axes(2)+0.5;
c2 = axes(3)+0.5;

hfig1 = figure(1);
lg(:,1)=plotellipsoid(center,axes(1),axes(2),axes(3),[0.2,0.2,0],.5);
hold on
lg(:,2)=plotellipsoid(center,a2,b2,c2,[0.83,0.82,0.78],0.5);
hold on;

% Demonstrations

for i = 1:nbDemo
    
    if i ==1
        lg(3) = plot3(demos{1,i}.X(1,1),demos{1,i}.X(2,1),demos{1,i}.X(3,1),'ok','MarkerSize',6,'LineWidth',3); hold on
        lg(4)=plot3(demos{1,i}.X(1,end),demos{1,i}.X(2,end),demos{1,i}.X(3,end),'*k','MarkerSize',18,'LineWidth',3); hold on
        lg(5)=plot3(demos{1,i}.X(1,:),demos{1,i}.X(2,:),demos{1,i}.X(3,:),'r-','LineWidth',3); hold on
    else
        plot3(demos{1,i}.X(1,1),demos{1,i}.X(2,1),demos{1,i}.X(3,1),'ok','MarkerSize',6,'LineWidth',3); hold on
        plot3(demos{1,i}.X(1,:),demos{1,i}.X(2,:),demos{1,i}.X(3,:),'r-','LineWidth',3); hold on
    end

end

%Sampled points

lg(6) = plot3(xtau(:,1),xtau(:,2),xtau(:,3),'o','MarkerSize',6,'Color',[0.6,0.6,0.6]);

% Estimated pos
for i =1:nbPoints
    lg(7) = plot3(estimatePos(1,:,i),estimatePos(2,:,i),estimatePos(3,:,i),'--','Color',[0.0,0.6,0.2],'LineWidth',4); hold on;
    plot3(estimatePos(1,1,i),estimatePos(2,1,i),estimatePos(3,1,i),'ok','MarkerSize',6,'LineWidth',3); hold on;
    plot3(estimatePos(1,end,i),estimatePos(2,end,i),estimatePos(3,end,i),'*k','MarkerSize',18,'LineWidth',3); hold on;
end

% Unconstrained Pos
% lg(8) = plot3(unconstrainedPos(1,:),unconstrainedPos(2,:),unconstrainedPos(3,:),'b--','LineWidth',3);
% hold on 
% plot3(unconstrainedPos(1,end),unconstrainedPos(2,end),unconstrainedPos(3,end),'*k','MarkerSize',18,'LineWidth',3);
% Logistics 
xlabel('$x \ [\mathrm{m}]$','Interpreter','latex')
ylabel('$y \ [\mathrm{m}]$','Interpreter','latex')
zlabel('$\psi \ [\mathrm{rad}]$','Interpreter','latex')

legend([lg(1),lg(5),lg(8),lg(7),lg(3),lg(4),lg(6)],'Invariant Region','Demos',...
'Unconstrained','Constrained','Initial Conditions','Target Location',...
'Sampled Points','Interpreter','latex','NumColumns',1,'Orientation','vertical','Location','northwest','FontSize', 32)

set(gca,'FontSize',32)

% perturbation_discretePush(demo_ind,demos, NonLin, WOpt,params)
% axis square
% axis([-2.5 2.5 -2.5 2.5])

% MagInset(hfig1, -1, [-2 -1 1 3], [2 3 2 4], {'NW','SW';'NE','SE'});
% set(gca,'FontSize',30)
