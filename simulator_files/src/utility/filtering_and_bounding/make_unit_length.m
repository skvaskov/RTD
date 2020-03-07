function v_out = make_unit_length(v_in)
% v_out = make_unit_length(v_in)
%
% Given an n-by-m array of column vectors v_in, return an n-by-m array of
% these same vectors but all of unit length.
    v_out = v_in ./ vecnorm(v_in) ;
end