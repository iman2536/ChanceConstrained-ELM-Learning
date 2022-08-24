function [] = perturbation_discretePush(demo_ind,demos, NonLin, WOpt,params)
%This function adds a discrete random push during the state reproduction of
%using the learned model. 
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2022 Iman Salehi, Robotics and Controls Lab        %%%
%%%                     ECE, UConn, Connecticut, USA                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n = 3; % dimension of the system
Nd = 5; % number of demos
lengthSpan = zeros(Nd,1);


drawArrow = @(x,y,z) quiver3(x(1),y(1),z(1),x(2)-x(1),y(2)-y(1),z(2)-z(1),0,'k','LineWidth',2,'MaxHeadSize',0.5); % a function to draw an arrow    
for k = 2:demo_ind
    
    curve_length = 0;
    for t = 2:size(demos{1,k}.X,2)
        x_temp = (demos{1,k}.X(1,t))-(demos{1,k}.X(1,t-1));
        y_temp = (demos{1,k}.X(2,t))-(demos{1,k}.X(2,t-1));
        z_temp = (demos{1,k}.X(3,t))-(demos{1,k}.X(3,t-1));
        temp_length = sqrt(x_temp^2+y_temp^2+z_temp^2);
        curve_length = curve_length + temp_length; 
    end 
    dt = demos{1, k}.dt;
    lengthSpan(k)= curve_length;
    time_duration(k) = length(demos{1,k}.X)*dt; % in seconds
    tp(k) = time_duration(k)*rand; % pertubation time instance (in seconds)
    tp_index(k) = floor(tp(k)/dt); % pertubation time index 
    
    MU(k) = 0.01* lengthSpan(k); 
    variance(k) = 0.01*lengthSpan(k);
    amplitude(k) = normrnd(MU(k),variance(k)); % perturbation amplitude
    r = -0.01 + 1*rand(n,1); % Generate a random number uniformly distributed random vector between (-0.5,0.5)
    vp(:,k) = amplitude(k)*(r./(norm(r,2))); % perturbation vector
    
    xhat{1,k}(:,1) = demos{1,k}.X(:,1);%+3*randn; % initial the reproduction
    
    t = 1; % intialize time index
    while true 
        NonLinDemoRepro_Perturb{1,k}(:,t)=feval(NonLin,demos{1,k}.X(1,t),demos{1,k}.X(2,t),demos{1,k}.X(3,t),...
                                                demos{1,k}.X(1,t)-params.NEWxstar(1),demos{1,k}.X(2,t)-params.NEWxstar(2));
%         NonLinDemoRepro_Perturb{1,k}(:,t)=feval(NonLin,xhat{1,k}(1,t),xhat{1,k}(2,t),xhat{1,k}(3,t),...
%                                                 xhat{1,k}(1,t)-params.xstar(1),xhat{1,k}(2,t)-params.xstar(2));
        xhat{1,k}(:,t+1) = WOpt'*NonLinDemoRepro_Perturb{1,k}(:,t);
        
%         xhat{1,k}(:,t+1) = xhat{1,k}(:,t)+ dt.*xdot_hat{1,k}(:,t);  
        
        if t+1 == tp_index(k)
            plot3(xhat{1,k}(1,t+1),xhat{1,k}(2,t+1),xhat{1,k}(3,t+1),'ko','MarkerSize',15);
            xhat{1,k}(:,t+1) = xhat{1,k}(:,t+1)+vp(:,k); % apply perturbation
            % Plotting an arrow showing the perturbation
            px = [xhat{1,k}(1,t),xhat{1,k}(1,t+1)]; 
            py = [xhat{1,k}(2,t),xhat{1,k}(2,t+1)];
            pz = [xhat{1,k}(3,t),xhat{1,k}(3,t+1)];
            drawArrow(px,py,pz);
            
        end
        if 999==t
            break;
        end
        t = t+1; % update time index
    end
    hold on
    plot3(xhat{1,k}(1,1:tp_index(k)-1),xhat{1,k}(2,1:tp_index(k)-1),xhat{1,k}(3,1:tp_index(k)-1),'g--','LineWidth',2);
    hold on; plot3(xhat{1,k}(1,tp_index(k)+1:end),xhat{1,k}(2,tp_index(k)+1:end),xhat{1,k}(3,tp_index(k)+1:end),'g--','LineWidth',2);
    hold on; plot3(xhat{1,k}(1,1),xhat{1,k}(2,1),xhat{1,k}(3,1),'ok','markersize',5,'linewidth',5);
    
end

