function v = get_5point_derivative(x,t)
%returns the numerical derivative of data x, with corresponding time vector
% t
%uses a 5 point finite difference formula in the interior points, and
%forward and backward different operators on the left and right edges

%input: x = dxN data
%       t = 1xN time 

%output: v = dxN vector of the numerical derivatives

%Author: Sean Vaskov
%Created: Oct 2018

%%
N=length(t);
v=NaN(size(x,1),N);

for i=[1,2,N-1]
v(:,i)=(x(:,i+1)-x(:,i))./repmat((t(i+1)-t(i)),[size(x,1),1]);
end

for i=3:N-2
    v(:,i)=(-x(:,i+2)+8*x(:,i+1)-8*x(:,i-1)+x(:,i-2))./repmat((12*(t(i)-t(i-1))),[size(x,1),1]);
end

v(:,N)=(x(:,N)-x(:,N-1))./repmat((t(N)-t(N-1)),[size(x,1),1]);
end

