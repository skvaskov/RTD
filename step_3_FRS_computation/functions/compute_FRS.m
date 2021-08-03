function out = compute_FRS(prob)
% OUTPUT = COMPUTE FRS(PROBLEM_STRUCTURE)
%
% An implementation of Program (D^l) from "Bridging the Gap Between Safety
% and Real-Time Performance in Receding-Horizon Trajectory Design for
% Mobile Robots." See Section 3; the program is on pg. 12.
%
% Paper: https://arxiv.org/abs/1809.06746
%
% The FRS, or Forward Reachable Set, is the set of all states that a system
% can reach in some time horizon T, with a dynamic model f, which we call a
% trajectory-producing model. This "low dimensional" model generates
% desired trajectories for a robot to track; since tracking isn't perfect,
% we also introduce a model g of the tracking error. The dynamics of f and
% g combined express the motion of the robot through its state space.
%
% This function is used to compute the FRS with sums-of-squares (SOS)
% polynomials. It calls the MOSEK conic optimization solver through the
% spotless SOS programming toolbox.
%
% The input structure must have the following fields:
%   t       time (1 x 1 msspoly)
%   z       state (n x 1 msspoly)
%   k       traj parameters (m x 1 msspoly)
%   f       traj-producing dynamics (n x 1 msspoly in t, z, and k)
%   hZ      vector of msspoly that are positive on the state space
%   hK      vector of msspoly that are positive on the parameter space
%   hZ0     vector of msspoly that are positive on initial states
%   degree  even number that is degree of SOS FRS approximation
%   cost    cost function on Z x K (usually the Lebesgue integral on Z x K)
%
% The input structure can contain optional fields:
%   g           tracking error dynamics (n x 1 msspoly in t, z, and k)
%   hT          1 x 1 msspoly that defines the time horizon T
%   FRS_states  states for w function (usually [z;k])
%   hFRS_states vector of msspoly that are positive for domain of w function (usually [hZ;hk])
%   v_lowerlim  upper limit for v function

% Authors: Shreyas Kousik and Sean Vaskov
% Created: 12 Apr 2019
% Updated: 10 Mar 2020

%% parse inputs
    disp('Extracting parameters')
    t = prob.t ; % time
    z = prob.z ; % states
    k = prob.k ; % trajectory parameters
    f = prob.f ; % dynamics
    hZ = prob.hZ ; % state space as a semi-algebraic set
    hK = prob.hK ; % param space as a semi-algebraic set
    hZ0 = prob.hZ0 ; % initial conds as a semi-algebraic set
    degree = prob.degree ; % degree of w polynomial
    cost = prob.cost ; % cost function (integral of w(z,k) over Z x K)

    % time horizon as a semi-algebraic set (default is t \in [0,1])
    if isfield(prob,'hT')
       hT = prob.hT ;
    else
       hT = t * (1 - t) ;
    end

    % subset of z \in Z to be used for w; e.g., if you have (x,y,theta) but
    % you only want w(x,y,k) to be your FRS polynomial, then set
    %   prob.FRS_states = [x;y]
    if isfield(prob,'FRS_states')
        FRS_states = prob.FRS_states ;
    else
        FRS_states = [z ;k];
    end


    if isfield(prob,'hFRS_states')
        hFRS_states = prob.hFRS_states ;
    else
        hFRS_states = [hZ ;hK];
    end
    
   
    if isfield(prob,'vproj_states')
        vproj_states = prob.vproj_states ;
    else
        vproj_states = [];
    end


    if isfield(prob,'hvproj_states')
        hvproj_states = prob.hvproj_states ;
    else
        hvproj_states = [];
    end

    % tracking error function g (default is to not have g, so we don't need
    % to compute the q decision variable, which makes the offline FRS
    % computation less memory-intensive)
    if isfield(prob, 'g')
        g = prob.g ;
    end
    
    %if we have states or parameters we dont want in the reachability
    %computation
    if isfield(prob,'x')
        x = prob.x;
    else
        x = [];
    end
    

%% define variables
    disp('Defining problem variables')

    % initialize program and indeterminate vars
    prog = spotsosprog;
    prog = prog.withIndeterminate(t) ;
    prog = prog.withIndeterminate(z) ;
    if ~isempty(k)
        prog = prog.withIndeterminate(k) ;
    end
    if ~isempty(x)
        prog = prog.withIndeterminate(x);
    end

    % create monomials for (v,w,q) polynomials
    vmon = monomials([t;z;k], 0:degree) ;
    wmon = monomials(FRS_states, 0:degree) ;

    % create (v,w,q) decision variable polynomials
    [prog, v, ~] = prog.newFreePoly(vmon) ;
    [prog, w, wcoeff] = prog.newFreePoly(wmon) ;
     if ~isempty(vproj_states)
        vprojmon = monomials(vproj_states,0:degree);
        [prog, vproj, ~] = prog.newFreePoly(vprojmon) ;
     end

    if exist('g','var')
        q = msspoly(zeros([size(g,2),1])) ;
        qmon = monomials([t;z;k], 0:degree) ;
        for qidx = 1:length(q)
            [prog, q(qidx), ~] = prog.newFreePoly(qmon) ;
        end
    end

    % create variables for the equality constraints of the program (D)
    if isfield(prob,'t0')
        t0 = prob.t0;
    else
        t0 = 0 ;
    end
    v0 = subs(v,t,t0) ;
    dvdt = diff(v,t) ;
    dvdz = diff(v,z) ;
    Lfv = dvdt + dvdz*f ;
    if exist('g','var')
        Lgv = dvdz*g ;
    end

%% define constraints
    disp('Defining constraints')
    tic

    % if tracking error is included, we need the following constraints:
    if exist('g','var')
        % -Lfv - sum(q) > 0 on T x Z x K
        prog = sosOnK(prog, -Lfv - ones(size(q))'*q, [t;z;k], [hT; hZ; hK], degree) ;
    else
        % -Lfv > 0 on T x Z x K
        prog = sosOnK(prog, -Lfv, [t;z;k], [hT; hZ; hK], degree) ;
    end

    if isempty(vproj_states)
        % v(t,.) + w > 1 on T x Z x K
        prog = sosOnK(prog, v + w - 1, unique([t;z;FRS_states;k]), unique([hT;hFRS_states; hZ; hK]), degree) ;
    else
        prog = sosOnK(prog,  v+vproj, unique([t;z;vproj_states;k]), unique([hT;hvproj_states; hZ; hK]), degree) ;

        prog = sosOnK(prog,  w-vproj - 1, [t;vproj_states;k], [hT;hvproj_states; hK], degree) ;
    end
    
    % w > 0 on hFRS states
    prog = sosOnK(prog, w, FRS_states, hFRS_states, degree) ;

    % -v(0,.) > 0 on Z0 x K
    prog = sosOnK(prog, -v0, [z;k], [hZ0; hK], degree) ;
    
    if isfield(prob,'v_lowerlim')
        prog = prog.withSOS(v-prob.v_lowerlim);
    end
    
    if isfield(prob,'hBoundary')
        hBoundary = prob.hBoundary;
        for i = 1:length(hBoundary)
            if ~isempty(vproj_states)
                prog = sosOnK(prog, -vproj ,[t;vproj_states;k], [hT; hBoundary{i}; hK], degree) ;
                
            else
                prog = sosOnK(prog, v ,[t;z;k], [hT; hBoundary{i}; hK], degree) ;
            end
        end
    end

    % if tracking error is included, we need the following constraints:
    % q - Lgv > 0 on T x Z x K
    % q + Lgv > 0 on T x Z x K
    % q > 0 on T x Z x K
    if exist('g','var')
        for qidx = 1:length(q)
            prog = sosOnK(prog, q(qidx) - Lgv(qidx), [t;z;k], [hT;hZ;hK], degree) ;
            prog = sosOnK(prog, q(qidx) + Lgv(qidx), [t;z;k], [hT;hZ;hK], degree) ;
            prog = sosOnK(prog, q(qidx), [t;z;k], [hT;hZ;hK], degree) ;
        end
    end

    toc

%% create problem object for computation
    disp('Setting up FRS problem')
    obj = cost(wmon)' * wcoeff ;
    out.prog = prog ;
    out.wmon = wmon ;
    out.wcoeff = wcoeff ;
    out.obj_func = obj ;
    options = spot_sdp_default_options() ;
    options.verbose = 1 ;
    options.domain_size = 1;
    options.solveroptions = [];

%% solve for FRS
    disp('Solving for the FRS')
    start_tic = tic ;
    sol = prog.minimize(obj, @spot_mosek, options);
    end_time = toc(start_tic) ;

%% extract problem info
    disp('Extracting results')
    v_out = sol.eval(v) ;
    w_out = sol.eval(w) ;

    out.v = v_out ;
    if ~isempty(vproj_states)
        out.vproj = sol.eval(vproj);
    end
    out.lyapunov_function = v_out ;
    out.w = w_out ;
    out.indicator_function = w_out ;
    out.final_cost = sol.eval(obj) ;
    out.duration = end_time ;
    out.input_problem = prob ;

    disp([num2str(end_time/3600), ' hrs elapsed solving problem'])
end
