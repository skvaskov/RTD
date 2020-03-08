function [z_scale,z_offset,Zsim,Tsim] = get_state_scaling_factors(f_fun,Z0_range,varargin)

%input 
%dynamics_fun: function handle (t,z,k) that returns the systems dynamics
%Z0_range: N_z x 2 vector with initial condition range [min,max]

%varargin:
N=1000; %number of random combos of (z,k) in Z0_range and K_range to generate
T=1; %Time to forward integrate
dt=0.1; %time discretization to check for scaling
n_scale = 0.6*ones(size(Z0_range,1),1); %box to scale to (typically less than 1)
plotting = false; %plotting on or off?
g = []; %function handle g:TxXxK to N_z x N_d matrix, for disturbance terms dz/dt = f + g*d, d(t) in [-1,1]
N_d = 0;
hZ0 = []; %no semialgebriac sets defining hZ0, make this a cell of function handles of (z,k) if you want to include them
t_box_times = [];
footprint = []; %2xnfp points along vehicle rigid body you want to include
pose_indexs = [1;2;3];
K_range = [];

%what this function does
%gives scaling factors for space such that|(z(t)+z_offset)/z_scale|<n_scale
%where z(t) = int_0^t dynamics_fun(t,z,k) dt, and t is in [0,T], and z(0)
%and k are sampled from Z0_range and K_range

%output (if not t_box_times given)
%z_offset: n_z x 1 double satisfying the above inequality
%z_scale: n_z x 1double satisfying the above inequality

%output (if t_box_times given)
%z_offset(:,1): n_z x 1 double satisfying the above inequality for all t
%z_scale(:,1): n_z x 1double satisfying the above inequality for all t
%z_offset(:,i+1): n_z x 1 double satisfying the above inequality for t =
%t_box_times(i)
%z_scale(:,i+1): n_z x 1double satisfying the above inequality for t =
%t_box_times(i)

for idx = 1:2:length(varargin)
    switch varargin{idx}
        case 'N'
            N = varargin{idx+1};
        case 'T'
            T = varargin{idx+1};
        case 'dt'
            dt = varargin{idx+1};
        case 'n_scale'
            n_scale = varargin{idx+1};
        case 'plotting'
            plotting = varargin{idx+1};
        case 'g'
            g = varargin{idx+1};
        case 'N_d'
            N_d = varargin{idx+1};
        case 'hZ0'
            hZ0 = varargin{idx+1};
        case 't_box'
            t_box_times = varargin{idx+1};
        case 'footprint' 
            footprint = varargin{idx+1};
        case 'pose_idxs'
            pose_indexs = varargin{idx+1};
        case 'K_range'
            K_range = varargin{idx+1};
    end
end

N_z = size(Z0_range,1);
N_k = size(K_range,1);

if numel(n_scale) == 1
    n_scale = n_scale*ones(N_z,1);
end

%if we have a disturbance function add it to the parameters and function
if ~isempty(g)
    if N_d<=0
        if N_k>0
            g_test = g(rand,rand(N_z,1),rand(N_k,1));
        else
            g_test = g(rand,rand(N_z,1));
        end
        N_d = size(g_test,2);
    end
    
    K_range = [K_range;[-ones(N_d,1),ones(N_d,1)]];
    
      
    N_k = N_k+N_d;
    
    if N_k-N_d>0
        dynamics_fun = @(t,z,k) f_fun(t,z,k(1: (N_k-N_d) ) )+g(t,z,k(1:(N_k-N_d)))*k((N_k-N_d+1):end,1);
    else
        dynamics_fun = @(t,z,k) f_fun(t,z)+g(t,z)*k(1:end,1);
    end
  
else
    dynamics_fun = f_fun;
end

%generate random combinations of z(0) and k in rectangluar sets
L = combinator(2,sum(N_k+N_z))';

if N_k>0
combos_k = NaN(N_k,size(L,2));
for i = 1:N_k
    combos_k(i,:) = K_range(i,L(i,:));
end
combos_k = [combos_k,randRange(K_range(:,1),K_range(:,2),[],[],1,N-size(L,2))];
else
    combos_k = [];
end

combos_z0 = NaN(N_z,size(L,2));
for i = 1:N_z
    combos_z0(i,:) = Z0_range(i,L(N_k+i,:));
end

combos_z0 = [combos_z0,randRange(Z0_range(:,1),Z0_range(:,2),[],[],1,N-size(L,2))];

%if we have semi algebraic sets, check that all combos satisfy them, then
%add more until we reach at least N combos

if ~isempty(hZ0)
    
    num_combos = N-1;
    
    time_in_loop = tic;
    
    while num_combos<N
        if N_k>0
            combos_k = [combos_k,randRange(K_range(:,1),K_range(:,2),[],[],1,N)];
        end
        combos_z0 =  [combos_z0,randRange(Z0_range(:,1),Z0_range(:,2),[],[],1,N)];
        
        
        for i = 1:length(hZ0)

            if (N_k-N_d)>0
                Lz0 = hZ0{i}(combos_z0,combos_k)>=0;
                combos_k = combos_k(:,Lz0);
            else
                Lz0 = hZ0{i}(combos_z0)>=0;
            end
            combos_z0 = combos_z0(:,Lz0);
        end
        num_combos = size(combos_z0,2);
        
        if toc(time_in_loop)>60
            warning('searching for points in semialg sets exceeded 1 min')
        end
        
    end
    
    %trim combo back down to N
    if N_k>0
        combos_k = combos_k(:,1:N);
    end
    combos_z0 = combos_z0(:,1:N);
end

%forward integrate system and store results
Tvec = linspace(0,T,round(T/dt));


Z=NaN(N_z,length(Tvec),N,1+size(footprint,2));


for i = 1:N
    if N_k>0
        [~,temp] = ode45(@(t,z)dynamics_fun(t,z,combos_k(:,i)),Tvec,combos_z0(:,i));
    else
        [~,temp] = ode45(@(t,z)dynamics_fun(t,z),Tvec,combos_z0(:,i));
    end
    if size(temp,1) < 10
        a 
    end
    Z(:,:,i,1) = temp';
end
Zsim = Z(:,:,:,1);
Tsim = Tvec;
Ksim = combos_k;

for i = 1:size(footprint,2)
    Z(:,:,:,i+1) = Z(:,:,:,1);
    for j = 1:size(Z,3)
        fp = repmat(footprint(:,i),[1,length(Tvec)]);
        fp_r = rotmat(Z(pose_indexs(3),:,j,i+1))*reshape(fp,[2*length(Tvec),1]);
        fp_r = reshape(fp_r,[2,length(Tvec)]);
        Z(pose_indexs(1:2),:,j,i+1) = fp_r+Z(pose_indexs(1:2),:,j,i+1);
    end
end

if ~isempty(t_box_times)>0
    Z_box_times = NaN(N_z,length(t_box_times),N,1+size(footprint,2));
    for i = 1:N
        for j = 1 : length(t_box_times)
            for h = 1 : 1+size(footprint,2)
                Z_box_times(:,j,i,h) = interp1(Tvec',Z(:,:,i,h)',t_box_times(j))';
            end
        end
    end 
else
    Z_box_times = [];
end

if plotting
    figure
    for i = 1:N_z
        subplot(N_z,1,i)
        hold on
        for h = 1 : 1+size(footprint,2)
            for j = 1 : N
                plot(Tvec,Z(i,:,j,h),'b')
            end
        end
    end
    xlabel('time (s)')
    sgtitle('Unscaled states')
end

z_offset = NaN(N_z,1+length(t_box_times));
z_scale = NaN(N_z,1+length(t_box_times));
    
%find max and mins and scaling factors

for i=1:N_z
    max_z = max(max(max(Z(i,:,:),[],2,'omitnan'),[],3),[],4);
    min_z = min(min(min(Z(i,:,:),[],2,'omitnan'),[],3),[],4);

    z_offset(i,1) = -(max_z+min_z)/2;
    z_scale(i,1) = (max_z-min_z)/(n_scale(i)*2);
    
    for j = 1:length(t_box_times)
        max_z = max(max(Z_box_times(i,j,:,:),[],3),[],4);
        min_z = min(min(Z_box_times(i,j,:,:),[],3),[],4);
        
        z_offset(i,j+1) = -(max_z+min_z)/2;
        z_scale(i,j+1) = (max_z-min_z)/(n_scale(i)*2);
    end
    
end


if plotting
    figure
    for i = 1:N_z
        subplot(N_z,1,i)
        hold on
        
        for j = 1:N
            for h = 1 : 1+size(footprint,2)
                plot(Tvec,(Z(i,:,j,h)+z_offset(i))/z_scale(i),'b')
            end
        end
        
        plot(Tvec,n_scale(i)*ones(size(Tvec)),'k--')
        hold on
        plot(Tvec,-n_scale(i)*ones(size(Tvec)),'k--')
    end
    xlabel('time (s)')
    sgtitle('scaled states')
end

end

