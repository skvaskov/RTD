classdef multi_link_agent < agent
    properties
        self_intersection_flag = false ;
    end
    
    methods
        function A = multi_link_agent(varargin)
            A@agent(varargin{:}) ;
        end
        
        function self_intersection_check(A,check_time)
            % A.self_intersection_check(check_time)
            %
            % Run a check for self-intersections and set the property
            % A.self_intersection_flag to "true."
            
            if nargin < 2
                check_time = A.time(end) ;
            end
            
            A.self_intersection_flag = false ;
        end
    end
end