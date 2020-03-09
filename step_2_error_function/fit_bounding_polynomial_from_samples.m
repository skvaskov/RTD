function p = fit_bounding_polynomial_from_samples(p_data,x_data,x,degree)

prog = spotsosprog();

prog = prog.withIndeterminate(x);

pterms = monomials(x,0:degree);

[prog,p,~] = prog.newFreePoly(pterms); 

err = msubs(p,x,x_data)-p_data;

prog=prog.withPos(err);

[prog,obj] = prog.newFree(1);

prog=prog.withPos(obj+err);
prog=prog.withPos(obj-err);

options = spot_sdp_default_options();
options.verbose = 1;

sol = prog.minimize(obj, @spot_mosek, options);

p = sol.eval(p);

end
