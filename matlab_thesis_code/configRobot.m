function params=configRobot(params)

%% config robot
params.Robot.xmax=100;  % limit of torus on x-axis in cm
params.Robot.ymax=100;  % limit of torus on y-axis in cm
params.Robot.xstart=20;
params.Robot.ystart=20;
params.Robot.thstart=0;
params.Robot.worldWrapped=false;
params.Robot.shape=ROBOTSHAPE.DOT;
params.Robot.sonarRange=20; 
params.Robot.sonarStartAngle=pi/10; % pi/20
params.Robot.Umin = 0.018;  % minimum wheel velocity in cm per ms - values: 0.015 (bw),  0.025, 0.05 (ms)
params.Robot.Rmax = 4; % estimated peak motor firing rate in Hz - values: 2, 40 (ms)
params.Robot.FI=8; % 5 for expparams.guiOn=true;onential % 8 training mult
params.Robot.dt=100;


end