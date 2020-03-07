function [varargout] = match_trajectories(tdes,varargin)
% [z1,z2,...] = match_trajectories(tdes,t1,z1,t2,z2,...,interp_type)
%
% Given some desired sample times and any number of time vectors (1-by-Nt)
% and associated trajectories (Nstates-by-Nt), resample the trajectories
% linearly at the desired times.

    varargout = cell(1,nargout) ;
    
    if ischar(varargin{end})
        interp_type = varargin{end} ;
    else
        interp_type = 'linear' ;
    end
    
    out_idx = 1 ;
    for idx = 1:2:(length(varargin)-1)
        t = varargin{idx} ;
        Z = varargin{idx+1} ;
        if length(t)==1 && tdes==t
            varargout{out_idx} = Z;
        else
            varargout{out_idx} = interp1(t(:),Z',tdes(:),interp_type)' ;
        end
        
        out_idx = out_idx + 1 ;
    end
end