function [v,W_in] = BIP(input,nh,ActFunc)

%This function estimates the slope "a" and bias "b" of the sigmoid function
%such that the desired distribution for the neurons output h = f(as+b) is
%realized. The return value of thie function can be incorporated into ELM.

%Input: input=Data input
%       nh=Number of hidden neurons

%Output: v = [a,b] for each neuron

%Author: Iman Salehi - adapted from the BIP algorithm introduced in Neumann
%and Steil; Neurocomputing 2013.

u = input;
inDim = size(u,1);
Ntr = size(u,2);

W_in=randn(nh,inDim); 
Bias=randn(nh,1);


for i = 1:nh
    
    for j = 1:Ntr
        s(i,j) = W_in(i,:)*u(:,j);%+Bias(i);
    end
    sorted_s(i,:) = sort(s(i,:),2);
    
%%fixed mu and sigma for the desired output distribution. 
%It is important that the virtual targets are in [0 1] if the sigmoid
%function is considered. [Ref: neumann2013optimizing]

    mu_x(i)= 0.5;
    sigma_x(i)=0.1;
    
%%draw targets t=(t1,t2 . . . t_Ntr) from the desired distribution f_des
   
    t(i,:)=normrnd(mu_x(i),sigma_x(i),1,Ntr);
    t(i,:)=sort(t(i,:),2);
    
%%Using Ordinary Least Square method to minimize ||\phi(s)v-f^{-1}(t)|| and
%%f is a tansigmoid function. 

if strcmp(ActFunc,'tansig')
    finv_t(i,:) = 1./( 2./(1+exp(-2*t(i,:)))-ones(1,Ntr));
elseif  strcmp(ActFunc,'logsig')
    finv_t(i,:)= 1+exp(-t(i,:));
else 
    dlX = dlarray(t(i,:));
    finv_t(i,:)= 1./extractdata(relu(dlX));
end
    phi=[sorted_s(i,:)',ones(1,Ntr)'];
    
    v(:,i) = inv(phi'*phi)*phi'*finv_t(i,:)';  
end
end







