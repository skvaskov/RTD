function [v] = get_5point_derivative(x,t)
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

