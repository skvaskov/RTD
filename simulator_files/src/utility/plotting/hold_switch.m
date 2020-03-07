function hold_check = hold_switch(hold_check_in)
% hold_check = hold_switch()
% hold_switch(hold_check)
%
% This function checks if "hold on" has already been called for the current
% axes. If not, then it calls "hold on" and returns a hold_check flag that
% tells the user whether or not it called "hold on". This same flag can be
% passed in to hold_switch to decide whether or not to call "hold off"
% later in a piece of code.
%
% Example: The following lines allow the user to turn hold on temporarily
% for just the two plot calls between the hold_switch calls.
%     hc = hold_switch() ;
%     plot(data_1)
%     plot(data_2)
%     hold_switch(hc)
%
% Author: Shreyas Kousik
% Created: 4 Nov 2019
% Updated: -

    if nargin < 1
        hold_check = ishold ;
        
        if ~hold_check
            hold on
        end
    else
        if ~hold_check_in
            hold off
        end
    end
end