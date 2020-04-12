function set_plot_linewidths(linewidth)
% set_plot_linewidths(linewidth)
%
% Set linewidths of all lines and patches to the provided linewidth
% (default is 1.5).
%
% Author: Shreyas Kousik
% Created: 29 Oct 2019
% Updated: -

    if nargin < 1
        linewidth = 1.5 ;
    end

    % get all lines
    h = findall(gcf,'Type','Line') ;
    for idx = 1:length(h)
        h(idx).LineWidth = linewidth ;
    end
    
    % get all patches
    h = findall(gcf,'Type','Patch') ;
    for idx = 1:length(h)
        h(idx).LineWidth = linewidth ;
    end
end