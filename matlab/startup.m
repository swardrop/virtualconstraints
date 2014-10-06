% Set graphing defaults
set(0,'defaultlinelinewidth',1)
set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on')
set(groot,'defaultAxesColorOrder',[0.4 0.4 0.4;0.2 0.2 0.2;0 0 0],...
    'defaultAxesLineStyleOrder','-|--|:|-.');

% Add critical folders to path
addpath model\cg\   % <-- Edit this to change model used
addpath worker