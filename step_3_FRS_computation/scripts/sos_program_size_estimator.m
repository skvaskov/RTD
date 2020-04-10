%% description

% This script estimates the number of free variables in program (D). 
% In our example, we have num_g g functions all of degree g_degree
% In our example, the domain is defined by 1 semialgebraic per dimension,
% all of degree semialg_degree

% Author: Sean Vaskov
% Created: 08 April 2020

% user parameters
n = 6; %system dimension (time, states, parameters)
l = 4; %relaxation degree (w,v,q will be degree 2*l)
dynamics_degree = 5; %max degree of dynamics, f
g_degree = [3,3]; %degree of g functions
semialg_degree = 2; %degree of semi algedbriac sets definining domain (typically 2 for rectangular domains)
w_dimension = n-2; %number of states in w, typically n-1
footprint_is_circle = false; %if the footprint is a circle, assume we have 1 semi algebraic set for x and y in HZ0

%% automated from here
num_g = length(g_degree);

%number of free variables from decision polynomials
Ndv = (2+num_g-1)*nchoosek(2*l+n,n)+nchoosek(2*l+w_dimension,w_dimension);

%determine degree of polynomial in each constraints
constraint_degree =    [dynamics_degree+2*l-1, repmat( g_degree+2*l-1 ,[1, 2] ), repmat( 2*l,[1, num_g+3])];

%determine dimension of each constraint
constraint_dimension = [n*ones(1, 1 + 2*num_g + num_g), n-1, n, w_dimension]; 

%assume 1 semialg set of degree semialg_degree per each dimension
n_semialg_constriants = constraint_dimension;

%if the footprint is a circle assume we have 1 less semi alg set in the
%constraint for t = 0
if footprint_is_circle
    n_semialg_constriants(end-2) = n-2;
end

%determine number of free variables for s polynomials in each constraint
Ncons = NaN(size(constraint_degree));
for i = 1:length(constraint_degree)
    %modify degree of s variable to be even (see sosOnK function)
%     cd = mod(constraint_degree(i),2) + constraint_degree(i);
    cd = constraint_degree(i)-semialg_degree;
    cd = max(0,mod(cd,2)+cd);
    
    Ncons(i) =  n_semialg_constriants(i)*nchoosek(cd+constraint_dimension(i),constraint_dimension(i));
    
end

%add free variables from decision and constraint polynomials
Nfree = Ndv + sum(Ncons);

disp(['Estimated Free Variables: ',   num2str(Nfree)])
