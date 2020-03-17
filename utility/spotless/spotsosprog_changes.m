% TODO:
%  -- test basis for dep. on indeterminates only.
%
classdef spotsosprog < spotprog
    properties
        sosExpr = {};
        sosTrigExpr = {};
        dsosExpr = {};
        sdsosExpr = {};
	diagsosExpr = {};
        indeterminates = msspoly([]);
        gramMatrices = {};
        gramMonomials = {};
    end

    methods (Access = private)
        function name = indetName(pr)
            name = [ pr.name 'x'];
        end
        
        
        function flag = isPolyInIndet(pr,exp)
            mtch = match(pr.indeterminates,decomp(exp));
            flag =  all(mtch ~= 0);
        end
        
        function [flag,indet] = isRealPolyLinearInDec(pr,exp)
            [x,pow,Coeff] = decomp(exp);
            
            mtch = match(x,pr.variables);
            
            flag = ~(any(imag(Coeff(:)) ~= 0) | ...
                     any(pow(:,mtch(mtch~=0))<0) | ...
                     any(sum(pow(:,mtch(mtch~=0)),2)>1));

            if ~flag,
                indet = [];
            else
                mtch = match(pr.variables,x);
                indet = x(find(mtch==0));
                if any(istrig(indet)) || ~pr.isPolyInIndet(indet)
                    flag = 0;
                    indet = [];
                end
            end
        end
        
        % function [flag,tIn,pIn] = trigPolyLinearInDec(pr,expr)
        %     [x,pow,Coeff] = decomp(expr);
            
        %     mtch = match(x,pr.variables);
            
        %     % TODO: conj. symmetric check.
        %     flag = ~(any(any(pow(:,mtch(mtch~=0)) > 1)));
            
        %     if ~flag, 
        %         indet = [];
        %     else
        %         mtch = match(pr.variables,x);
        %         indet = x(find(mtch==0));
        %         msk = istrig(indet);
        %         tIn = indet(find(msk));
        %         pIn = indet(find(~msk));
        %     end
        % end
    end
    

    
    methods (Access = public)
        function [pr,Q,phi,y,basis,eqMultFac] = buildSOSDecompPrimal(pr,expr,newGram,options)
            if ~spot_hasSize(expr,[1 1])
                error('buildSOSDecomp expects a scalar polynomial.');
            end
            decvar = pr.variables;
            
            % Rescale expr to normalize things.
            A0 = diff(expr,decvar);
            b0 = subs(expr,decvar,0*decvar);
            [~,~,Coeff] = decomp([b0 A0].');
            
            eqMultFac = 1;
            
%             if nnz(Coeff) == 0
%                 Q = [];
%                 phi = [];
%                 y = [];
%                 basis = [];
%             else
%                 expr = expr/max(abs(Coeff(:)));
%                 eqMultFac = max(abs(Coeff(:)));
%             end
%             
            
            % Build the Gram basis
            phi = spotsosprog.buildGramBasis(expr,decvar,options); 
            
            [pr,Q] = newGram(pr,length(phi));
            sosCnst = expr-phi'*Q*phi;
            
            decvar = pr.variables;
            
            A = diff(sosCnst,decvar);
            b = subs(sosCnst,decvar,0*decvar);
            [var,pow,Coeff] = decomp([b A].');
            
            [pr,y] = pr.withEqs(Coeff'*[1;decvar]);
            basis = recomp(var,pow,speye(size(pow,1))); 
        end
        
        function [pr,Q,phi,y,basis] = buildDSOSDecompPrimal(pr,expr,options)
            if ~spot_hasSize(expr,[1 1])
                error('buildSOSDecomp expects a scalar polynomial.');
            end

            decvar = pr.variables;

            phi = spotsosprog.buildGramBasis(expr,decvar,options);

            [pr,Q] = pr.newDD(length(phi));
    
            decvar = [decvar ; mss_s2v(Q)];
            sosCnst = expr-phi'*Q*phi;

            A = diff(sosCnst,decvar);
            b = subs(sosCnst,decvar,0*decvar);
            [var,pow,Coeff] = decomp([b A].');
            
            [pr,y] = pr.withEqs(Coeff'*[1;decvar]);
            basis = recomp(var,pow,speye(size(pow,1)));
        end
        
        function [pr,Q,phi,y,basis] = buildSOSDecompDual(pr,expr)
            if ~spot_hasSize(expr,[1 1])
                error('buildSOSDecomp expects a scalar polynomial.');
            end
            
            y = pr.variables;
            
            phi = spotsosprog.buildGramBasis(expr,y);
            

            %  This next code requires that phi be the monomials
            %
            %  q in PSD,  C - A(y) in K,   D(y) + E(q) = f.
            %
            %  Note E has full row rank.  Pick q = q0 + G1.y + G2.z s.t.:
            %
            %  E(q0) = f,   D + E.G1 = 0,   E.G2 = 0.  
            %
            %  G2 w/ linearly indep. cols.
            %

            
            % Introuduce /dummy/ semidefinite variables.
            [~,Q] = pr.newPSD(length(phi));            
            q = mss_s2v(Q);
            
            decvar = [y ; q];
            sosCnst = phi'*Q*phi-expr;

            % A1*y + A2*q = b.
            A1 = diff(sosCnst,y);
            A2 = diff(sosCnst,q);
            b = -subs(sosCnst,[y;q],0*[y;q]);
            [var,pow,Coeff] = decomp([b A1 A2].');
            
            Coeff = Coeff.';
            
            f  = Coeff(:,1);
            D = Coeff(:,1+(1:length(y)));
            E = Coeff(:,1+length(y)+(1:length(q)));
        
            
            ny = length(y);
            m = length(f);
            nq = length(q);
            
            %  Now we'll use some of the structure of E.
            [row,col,s] = find(E);
            
            if ~all(col == (1:nq)') || length(unique(row)) ~= m
                error('Basis assumptions violated for SOS decomposition.');
            end
            
            % Scaling matrix.
            S = sparse(col,col,1./s,nq,nq);
            
            [row,I] = sort(row);
            S = S(:,I);

            
            % Find representatives for each column.
            [rr,ii,~] = unique(row);
            [~,I] = sort(rr);
            col_of_Id = col(ii(I));
            
            % E(i,col_of_ID(i)) = 1, so setting v(col_of_ID) = b, zeros o.w. makes
            % (Ev) = b.
            
            % Construct the particular solution.
            q0 = S*sparse(col_of_Id,ones(m,1),f,nq,1);
            
            % Construct G1 that is m-by-ny so that D = - E.G1
            G1 = S*sparse(repmat(col_of_Id,ny,1),...  
                        reshape(repmat(1:ny,m,1),[],1),...
                        -D(:),nq,ny);
            
            % Construct G2 with (nq - m) linearly independent rows.
            G2 = speye(nq);
            
            % Find when col(i) ~= col(i-1)
            paired = col_of_Id(row);
            %paired = [ col(2:end) ; col(end)];
            
            G2 = G2 - sparse(paired,col,ones(nq,1),nq,nq);
            G2(:,col_of_Id) = [];
            G2 = S*G2;
            
            if nq > m
                [pr,z] = pr.newFree(nq-m);
                Q = mss_v2s(q0 + G1*y + G2*z);

            else
                Q = mss_v2s(q0 + G1*y);
            end
            pr = pr.withPSD(Q);
            basis = recomp(var,pow,speye(size(pow,1)));
        end
    end
    
    methods (Access = private)
        function [pr,x] = newPrivateIndeterminate(pr,no)
            x=msspoly(pr.indetName,no);
            [pr] = pr.withPrivateIndeterminate(x);
        end
        
        function pr = withPrivateIndeterminate(pr,var)
            [vid,f] = isfree(var);
            if ~f,
                error('Indeterminates must be free msspolys.');
            end
            skip=match(pr.indeterminates,var);
            pr.indeterminates = [ pr.indeterminates ; var(skip == 0)];
        end
    end
    
    methods
        function pr = spotsosprog(varargin)
            pr@spotprog(varargin{:});
        end
        function [pr,x] = newIndeterminate(pr,name,number)
            if nargin < 3, number = 1; end
            if name(1) == pr.name
                error(['Indeterminate name conflicts with program ' ...
                       'name.']);
            end
            x = msspoly(name,number);
            pr = pr.withIndeterminate(x);
        end
        
        function pr = withIndeterminate(pr,var)
            [vid,f] = isfree(var);
            if ~f,
                error('Indeterminates must be free msspolys.');
            end
            
            nm = name(var);
            for i = 1:length(nm)
                if nm{1}(1) == pr.name
                    error(['Indeterminate name conflicts with program ' ...
                           'name.']);
                end
            end
            pr = withPrivateIndeterminate(pr,var);
        end
        
        function [pr,poly,coeff] = newSOSPoly(pr,basis,n)
            if nargin < 3, n = 1; end
            [pr,poly,coeff] = newFreePoly(pr,basis,n);
            pr = pr.withSOS(poly);
        end
        
        function [pr,poly,coeff] = newDSOSPoly(pr,basis,n)
          if nargin < 3, n = 1; end
          
          [pr,poly,coeff] = newFreePoly(pr,basis,n);
          pr = pr.withDSOS(poly);
        end
        
        function [pr,poly,coeff] = newSDSOSPoly(pr,basis,n)
          if nargin < 3, n = 1; end
          [pr,poly,coeff] = newFreePoly(pr,basis,n);
          pr = pr.withSDSOS(poly);
        end
        
        function [pr,conInd] = withSOS(pr,expr)
            if ~pr.isRealPolyLinearInDec(expr)
                error(['Coefficients must be real, indeterminates ' ...
                       'non-trigonometric, and expression must ' ...
                      'be linear in decision variables.']);
            end
            tokens = length(pr.sosExpr) + (1:prod(size(expr)));
            pr.sosExpr = [ pr.sosExpr ; expr(:) ];
            conInd = pr.numSOS;
        end
        
        function [pr,conInd] = withDSOS(pr,expr)
            if ~pr.isRealPolyLinearInDec(expr)
                error(['Coefficients must be real, indeterminates ' ...
                       'non-trigonometric, and expression must ' ...
                      'be linear in decision variables.']);
            end
            tokens = length(pr.dsosExpr) + (1:prod(size(expr)));
            pr.dsosExpr = [ pr.dsosExpr ; expr(:) ];
            conInd = pr.numSOS + pr.numDSOS;
        end
        
        function [pr,conInd] = withSDSOS(pr,expr)
            if ~pr.isRealPolyLinearInDec(expr)
                error(['Coefficients must be real, indeterminates ' ...
                       'non-trigonometric, and expression must ' ...
                      'be linear in decision variables.']);
            end
            tokens = length(pr.sdsosExpr) + (1:prod(size(expr)));
            pr.sdsosExpr = [ pr.sdsosExpr ; expr(:) ];
            conInd = pr.numSOS + pr.numDSOS + pr.numSDSOS;
        end

        function [pr,conInd] = withDiagSOS(pr,expr)
            if ~pr.isRealPolyLinearInDec(expr)
                error(['Coefficients must be real, indeterminates ' ...
                       'non-trigonometric, and expression must ' ...
                      'be linear in decision variables.']);
            end
            tokens = length(pr.diagsosExpr) + (1:prod(size(expr)));
            pr.diagsosExpr = [ pr.diagsosExpr ; expr(:) ];
            conInd = pr.numSOS + pr.numDSOS + pr.numSDSOS + pr.numDiagSOS;
        end
        
        function [pr] = withSOSMatrix(pr,expr)
            [lindec,indet] = pr.isRealPolyLinearInDec(expr);
            if ~lindec
                error(['Coefficients must be real, indeterminates ' ...
                       'non-trigonometric, and expression must ' ...
                      'be linear in decision variables.']);
            end
            
            if size(expr,1) ~= size(expr,2) || size(expr,1) == 0
                error('Expression must be a square non-empty matrix.');
            end
            
            [pr,q] = pr.newPrivateIndeterminate(size(expr,1));
            [pr] = pr.withSOS(q'*expr*q);
        end
        
        function [pr] = withPolyEqs(pr,expr)
            if ~pr.isRealPolyLinearInDec(expr)
                error(['Coefficients must be real, indeterminates ' ...
                       'non-trigonometric, and expression must ' ...
                       'be linear in decision variables.']);
            end

            expr = expr(:);
            decvar = pr.variables;
            
            [~,~,M] = decomp(expr,decvar);
            
            %monom = recomp(indet,pow,speye(size(pow,1)));
            
            [I,J,S] = find(M);
            
            [pr,y] = pr.withEqs(S);
            
            %basis = monom(J);
        end
        
        
        function n = numSOS(pr)
            n = length(pr.sosExpr);
        end
        
        function n = numDSOS(pr)
            n = length(pr.dsosExpr);
        end
        
        function n = numSDSOS(pr)
            n = length(pr.sdsosExpr);
        end

	function n = numDiagSOS(pr)
            n = length(pr.diagsosExpr);
        end
        
        function n = numTrigSOS(pr)
            n = length(pr.sosTrigExpr);
        end
        
        function [pr,poly,coeff] = newFreePoly(pr,basis,n)
            if nargin < 3, n = 1; end
            if ~isempty(basis) && n > 0
                if ~pr.isPolyInIndet(basis)
                    error('Basis must be polynomial in the indeterminates.');
                end
                [pr,coeff] = pr.newFree(length(basis)*n);
                poly = reshape(coeff,n,length(basis))*basis;
            else
                poly = [];
            end
        end
        
        
        function [sol, y, basis, dual_multiplier] = minimize(pr,varargin)
        %
        % sol = minimize(pr,pobj,solver,options)
        %
            if nargin >= 4,
                options = varargin{3};
            else
                options = spotprog.defaultOptions;
            end
            
            if ~isfield(options,'dualize')
                options.dualize = false;
            end
            
            if options.dualize
                error('Dualization not supported.');
                Q = cell(pr.numSOS,1);
                phi = cell(pr.numSOS,1);
                y   = cell(pr.numSOS,1);
                basis   = cell(pr.numSOS,1);
                dual_multiplier = ones(pr.numSOS,1);
                for i = 1:pr.numSOS
                    [pr,Q{i},phi{i},y{i},basis{i}] = pr.buildSOSDecompDual(pr.sosExpr(i));
                end
                
                sol = minimize@spotprog(pr,varargin{:});
            else
                dsosOff = pr.numSOS;
                sdsosOff = dsosOff + pr.numDSOS;
                diagsosOff = sdsosOff + pr.numDiagSOS;
                trigOff = diagsosOff + pr.numSDSOS;
                totalSOS = trigOff + pr.numTrigSOS;
                Q = cell(totalSOS,1);
                phi = cell(totalSOS,1);
                y   = cell(totalSOS,1);
                basis   = cell(totalSOS,1);
                dual_multiplier = ones(totalSOS,1);
                for i = 1:pr.numSOS
                    [pr,Q{i},phi{i},y{i},basis{i},eqMultFac] = pr.buildSOSDecompPrimal(pr.sosExpr(i),@newPSD,options);
                    pr.gramMatrices{i} = eqMultFac*Q{i};
                    pr.gramMonomials{i} = phi{i};
                    dual_multiplier(i) = eqMultFac;
                end
                for i = 1:pr.numDSOS
                    ioff = i + dsosOff;
                    if isfield(options,'basis_scale')
                      options.basis_scale_i = options.basis_scale{i};
                    end
                    [pr,Q{ioff},phi{ioff},y{ioff},basis{ioff},eqMultFac] = pr.buildSOSDecompPrimal(pr.dsosExpr(i),@newDD,options);
                    pr.gramMatrices{ioff} = eqMultFac*Q{ioff}; 
                    pr.gramMonomials{ioff} = phi{ioff};
                    dual_multiplier(ioff) = eqMultFac;
                end
                for i = 1:pr.numSDSOS
                    ioff = i + sdsosOff;
                    [pr,Q{ioff},phi{ioff},y{ioff},basis{ioff},eqMultFac] = pr.buildSOSDecompPrimal(pr.sdsosExpr(i),@newSDD,options);
                    pr.gramMatrices{ioff} = eqMultFac*Q{ioff};
                    pr.gramMonomials{ioff} = phi{ioff};
                    dual_multiplier(ioff) = eqMultFac;
                end
                for i = 1:pr.numDiagSOS
                    ioff = i + diagsosOff;
                    [pr,Q{ioff},phi{ioff},y{ioff},basis{ioff},eqMultFac] = pr.buildSOSDecompPrimal(pr.diagsosExpr(i),@newDiag,options);
                    pr.gramMatrices{ioff} = eqMultFac*Q{ioff};
                    pr.gramMonomials{ioff} = phi{ioff};
                    dual_multiplier(ioff) = eqMultFac;
                end
                for i = 1:pr.numTrigSOS
                    ioff = i + trigOff;
                    t = pr.sosTrigExpr{i};
                    [pr,Q{ioff},phi{ioff},y{ioff},basis{ioff}] = pr.buildSOSTrigDecomp(t{:});
                end
                
                sol = minimize@spotprog(pr,varargin{:});
            end
        end        
    end
    
    methods (Access = private, Static)
        function phi = buildGramBasis(expr,decvar,options)
            if ~spot_hasSize(expr,[1 1])
                error('buildGramBasis expects a scalar polynomial.');
            end
            
            [var,pow,M] = decomp(expr);
            mtch = match(var,decvar);
            b = 1:length(var);
            b(mtch(mtch ~= 0)) = [];
            indet = var(b);
            
            if length(indet) == 0
                phi = 1;
                return;
            end

            pow = pow(:,b); 

            exponent_m = spot_build_gram_basis(pow);
            
            phi = recomp(indet,exponent_m,speye(size(exponent_m,1)));
%             phi = monomials(indet, 0:max(max(exponent_m)));
            
            % Scaled monomial basis
            if ~isfield(options,'scale_monomials'); options.scale_monomials = false; end
            
            if options.scale_monomials
                % Compute degrees of monomials
                ds = sum(exponent_m,2);
                fds = factorial(ds);
                
                % Compute multinomial coefficients
                alphas = prod(factorial(exponent_m),2);
                
                % Compute scalings
                cs = sqrt(fds./alphas);
                
                % Apply scaling to phi vector
                phi = cs.*phi; 
            end
        
            if isfield(options,'basis_scale_i')
              phi = options.basis_scale_i*phi;
            end
        end
    end
end