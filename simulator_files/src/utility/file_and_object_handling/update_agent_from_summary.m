function A = update_agent_from_summary(A,summary)
% Given a summary output from the simulator and an agent A, set the agent's
% states, inputs, etc. according to the summary.
%
% Author: Shreyas Kousik
% Created: 3 Dec 2019
% Updated: -

AI = summary.agent_info ;
F = fieldnames(AI) ;
for idx = 1:length(F)
    f = F{idx} ;
    try
        A.(f) = AI.(f) ;
        disp([f,' filled'])
    catch
        disp([f, ' is not a valid property of the agent'])
    end
end
end