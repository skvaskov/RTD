function R = rotation_matrix_2D(h, sparse_flag)
% Given an angle in radians h, produce the 2-by-2 rotation matrix R that
% can rotate a point in \R^2 by the angle h about the origin
%
% If h is a vector (1-by-n or n-by-1), return a 2n-by-2n matrix in which
% each 2-by-2 block on the diagonal corresponds to each entry of h. This
% matrix is returned as a double if h has up to 100 entries, and a sparse
% matrix otherwise. Sparse output can also be forced by the argument
% sparse_flag.
    if nargin < 2
        sparse_flag = false ;
    end

    n = length(h) ;
    if n == 1
        R = [cos(h) -sin(h) ; sin(h) cos(h)] ;
    else
        h = h(:)' ;
        ch = cos(h) ; sh = sin(h) ;
        Rtall = [ch ; sh ; -sh ; ch] ;
        Rlong = reshape(Rtall,2,[]) ;
        if n <= 100 && ~sparse_flag
            % for smallish matrices, don't return sparse
            Rbig = repmat(Rlong, length(h),1) ;
            Icell = repmat({ones(2)},1,size(Rbig,2)/2,1) ;
            II = blkdiag(Icell{:}) ;
            R = Rbig.*II ;
        else
            io = 1:2:(2*n) ; % odd indices
            ie = 2:2:(2*n) ; % even indices
            
            % make row indices
            rio = repmat(io,2,1) ;
            rie = repmat(ie,2,1) ;
            rstack = [rio(:) rie(:)]' ;
            r = rstack(:) ;
            
            % make column indices
            c = repmat(1:2*n,2,1) ;
            c = c(:) ;
            
            % create sparse matrix
            R = sparse(r,c,Rlong(:)) ;
        end
    end
end