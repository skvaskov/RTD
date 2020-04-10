function FRS = get_segway_FRS_from_v_0(v_0,degree)
% FRS = get_segway_FRS_from_v_0(v_0, degree)
%
% Given an initial speed, load the corresponding FRS file for the segway.
% The second input (degree) should be 6, 8, 10, or 12. By default it is 10.
%
% Author: Shreyas Kousik
% Created: 9 Apr 2020

    % get degree
    if nargin < 2
        degree = 10 ;
    end

    % load FRS depending on v_0
    v_0_min = [0.0 0.5 1.0] ;
    v_0_max = [0.5 1.0 1.5] ;

    % figure out which interval the given v_0 is in
    idxs = (v_0 >= v_0_min) & (v_0 <= v_0_max) ;
    idx = find(idxs,1,'last') ;

    % return the corresponding v_0 range
    v_0_range = nan(1,2) ;
    v_0_range(1) = v_0_min(idx) ;
    v_0_range(2) = v_0_max(idx) ;

    switch v_0_range(1)
        case 0.0
            FRS = load(['segway_FRS_deg_',num2str(degree),'_v_0_0.0_to_0.5.mat']) ;
        case 0.5
            FRS = load(['segway_FRS_deg_',num2str(degree),'_v_0_0.5_to_1.0.mat']) ;
        case 1.0
            FRS = load(['segway_FRS_deg_',num2str(degree),'_v_0_1.0_to_1.5.mat']) ;
        otherwise
            error('Please pick a valid initial speed!')
    end
end