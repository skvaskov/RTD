function visualize_2D_FRS(solver_output,k_eval,w_color,figure_number,show_extra_contours)
    %% parse inputs
    if nargin < 5
        show_extra_contours = false ;
    end
    
    if nargin < 4
        figure_number = 1 ;
    end
    
    if nargin < 3
        % the color to be used for the 1-superlevel set of w
        w_color = 'b' ;
    end
    
    out = solver_output ;
    z = out.input_problem.z ;
    k = out.input_problem.k ;
    hZ0 = out.input_problem.hZ0 ;
    
    if nargin < 2
        k_eval = 2*rand(length(k),1) - 1 ;
    end
    
    %% get w polynomial
    % if (z,k) \in FRS, then w(x,k) >= 1
    w = out.w ;
    
    %% evaluate w on the choice of k
    wz = msubs(w,k,k_eval) ;
    
    %% set up for plotting
    % make a grid in the space Z x K ; note the Z and K ranges are hard
    % coded since the problem should always be scaled to live in [-1,1],
    % since we are optimizing over polynomials
    Z_range = [-1,1] ;
    N = 100 ; % grid density
    zvec = linspace(Z_range(1,1),Z_range(1,2),N) ;
    
    [Z1,Z2] = meshgrid(zvec, zvec) ;
    Z1Z2 = [Z1(:) Z2(:)]' ;
    
    % evaluate w on the grid
    w_eval = msubs(wz,z,Z1Z2) ;
    
    % reshape w for contour
    w_eval = reshape(full(w_eval),N,N) ;
    
    % evaluate hZ0 on the grid
    hZ0_eval = reshape(full(msubs(hZ0,z,Z1Z2)),N,N) ;
    
    %% plotting
    figure(figure_number) ; hold on ; axis equal ;
    
    % plot extra contours first so w can sit on top
    if show_extra_contours
        C2 = contour(Z1,Z2,w_eval,-1:0.25:1,'LineWidth',1) ;
        clabel(C2)
    end
    
    % plot w
    contour(Z1,Z2,w_eval,[1 1],'LineWidth',2,'EdgeColor',w_color) ;
    
    % plot hZ0
    contour(Z1,Z2,hZ0_eval,[0 0],...
        'LineStyle','--','LineWidth',2,'EdgeColor',[0 0.5 1]) ;
    
    % labeling
    xlabel('z_1')
    ylabel('z_2')
    
    set(gca,'FontSize',15)
    
end