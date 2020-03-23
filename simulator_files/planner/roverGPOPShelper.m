function [Tout,Uout,Zout] = roverGPOPShelper(gpops_problem)
    output = gpops2(gpops_problem) ;
    solution = output.result.solution ;
    Tout = solution.phase.time ;
    Uout = solution.phase.control ;
    Zout = solution.phase.state ;
end