%% Plot the results
%This script plots the result of the ELM parameter leanring of LASA
%handwriting motion dynamics. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% removing the offset - For the LASA dataset
nbDemo = 7;
% estimatePos = estimatePos-params.xstar;
% for i = 1:7
%     demos{1,i}.q=demos{1,i}.pos-params.xstar;
% end
% xtau = xtau - params.xstar';
% center = center - params.xstar';
% unconstrainedPos = unconstrainedPos - params.xstar;

%% Plots

hfig1 = figure(1);
[lg(1),bnds]=plotellipse(center, mjr_axis, mnr_axis, theta,[0.6,0.2,0],3);
hold on
lg(2)=plotellipse(center,mjr_axis+5, mnr_axis+5, theta,[0.2,0.1,0],1);
hold on;

% Demonstrations

for i = 1:nbDemo
    
    if i == 1
        lg(3) = plot(demos{1,i}.pos(1,1),demos{1,i}.pos(2,1),'ok','MarkerSize',6,'LineWidth',3); hold on
        lg(4)= plot(demos{1,i}.pos(1,end),demos{1,i}.pos(2,end),'*k','MarkerSize',18,'LineWidth',3); hold on
        lg(5) = plot(demos{1,i}.pos(1,:),demos{1,i}.pos(2,:),'r-','LineWidth',2); hold on;
    else 
        plot(demos{1,i}.pos(1,1),demos{1,i}.pos(2,1),'ok','MarkerSize',6,'LineWidth',3); hold on
        plot(demos{1,i}.pos(1,:),demos{1,i}.pos(2,:),'r-','LineWidth',2); hold on;
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

