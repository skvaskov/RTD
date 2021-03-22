%Normalize mss poly function so maximum coefficient (by abs value) is 1

%Sean Vaskov
%03/19/2021

function out = normalizeCoeffs(in)

out = msspoly(zeros(size(in)));

for i = 1:size(out,1)
    for j = 1:size(out,2)
        
        [x,p,M] = decomp(in(i,j));
        
        M = M/max(abs(M));
        
        out(i,j) = recomp(x,p,M);
    end
end

end