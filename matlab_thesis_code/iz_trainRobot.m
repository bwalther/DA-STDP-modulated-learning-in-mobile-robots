function iz_trainRobot(resultFileName,network)

%% config simulation
params.logtrace=false;
params.rwpunish=false;
params.stdpsym=false;
params.guiOn=false;
params.ads=false;
params.assignNetwork=false;

if nargin <1
    error('Specify resultFileName');
elseif nargin ==2
    params.network=network;
    params.assignNetwork=true;
end

ncontroller=iz_NController(params);
params=configSTDP(params);
params.T=60*60; %time in seconds
[res host]=unix('hostname');
host=strtrim(host);
slidingRmaxWindow=50;rmaxIdx=0;

%% config robot
params=configRobot(params);

%% init training controls
robotDirections={'left' 'straight' 'right' 'straight2'};
dir = struct(robotDirections{1},1,robotDirections{2},2,robotDirections{3},3,robotDirections{4},4);
cDir=dir.left;iterationFirings=[];
trainStepCount=1;
% shouldnt bee too much for training: short dist between reward and rewarding event. must reward turn
maxTrainStep=12;
extMotorCurrent=0.2;
extSensorCurrent=0.0;
trainMotorCurrentLeft=0.5; % starting Base:0.7, 0.55 After 2mins: 0.2 given weight diff: 1
trainMotorCurrentRight=0.5; %
rw=[];SLDist=0; SRDist=0;SCDist=0;missedCountLeft=0;missedCountRight=0;overshootLeft=0;overshootRight=0;
switchRw=false;

%% init robot environment
params.Robot.Rmax=4;
mrobot=Robot(params.Robot);
Env.xmax = params.Robot.xmax;
Env.ymax = params.Robot.ymax;
Env.wrapped=false;
Env.Obs=createObstacle(cDir);

%% init neuron controller
ncontroller.assignNetwork(params.network);
[SL SR ML MR IL IR N]=ncontroller.topologyInfo;
sensorSize=length(SL);
motorSize=length(ML);
MOVRmax=repmat(params.Robot.Rmax,slidingRmaxWindow,1);

%% init plotting
if params.guiOn
    f1=figure('Name','Training');
    xlim([0 Env.xmax]);
    ylim([0 Env.ymax]);
    hold on;
    for i = 1:length(Env.Obs)
        drawObstacle(Env.wrapped,Env.Obs(i),Env.xmax,Env.ymax);
    end
    rState = mrobot.getState();
    x= rState(1);
    y= rState(2);
    plot(x,y,'.')
    hold off;
    drawnow;
end
%% MAIN LOOP
tic;
for dsec=1:params.T*10
    
    %% calc sensor spike source current
    IExt=calcExternalInputCurrent;
    [HzPerGroup wDiffTurnLeft wDiffTurnRight]=ncontroller.simulationStep(dsec,rw,IExt);
    
    %% Read out motor firing rate
    HzML = HzPerGroup(dsec,3);
    HzMR = HzPerGroup(dsec,4);
    
    %mrobot.Rmax=max(mrobot.Rmax,max(HzML,HzMR)*0.9);
    if params.logtrace
        disp(['Hz: ' num2str(HzPerGroup(dsec,1)) ' -- ' num2str(HzPerGroup(dsec,2)) ' -- ' ...
            num2str(HzML) ' -- ' num2str(HzMR) ' -- ' ...
            num2str(HzPerGroup(dsec,5)) ' -- ' num2str(HzPerGroup(dsec,6)) ]);
    end
    
    d=ncontroller.serializePlotData;
    d.lastfirings(:,1)=d.lastfirings(:,1)+(100*(trainStepCount-1));
    iterationFirings=[iterationFirings;d.lastfirings];
    
    
    %% drive robot with motor neuron output
    mrobot.updatePosition(HzML,HzMR,Env.wrapped,false);
    trainStepCount=trainStepCount+1;
    rState = mrobot.getState();
    %% sensor reading  Testing(must do thi5s before drawing, so to not "draw over" obstacle)
    [TouchL TouchR TouchC SLDist SRDist SCDist Env] = RobotGetSensorsN(Env,rState(1),rState(2),rState(3),params);
    if (trainStepCount==maxTrainStep || TouchL || TouchR || TouchC ) % if collided or training finished
        %% create new obstacle, remove old one
        if  TouchL || TouchR || TouchC
            rw=[rw,(dsec+1)*100+1];
            if params.guiOn
                drawTouchedOb;
            end
        elseif cDir==dir.left
            missedCountLeft=missedCountLeft+1;
        elseif cDir==dir.right
            missedCountRight=missedCountRight+1;
        end
        adjustTrainingCurrent;
        %% reset training
        cDir=cDir+1;
        iterationFirings=[];
        if cDir >length(robotDirections)
            cDir=1;
        end
        Env.Obs=[createObstacle(cDir)];
        trainStepCount=1;
        mrobot.resetTraj();
    end
    if rmaxIdx>slidingRmaxWindow
        rmaxIdx=1;
    else
        rmaxIdx=rmaxIdx+1;
    end
    if ~mod(dsec,100)
        smr=sort(MOVRmax);
        mrobot.Rmax=max(smr(1:floor(0.9*slidingRmaxWindow)))*0.9;
    end
    %% plot robot
    if params.guiOn
        drawRobot(trainStepCount);
    end
    if ~mod(dsec,1000)
        telapsed=toc;
        tic;
        disp(['Simulated 1 minute. Time:' num2str(telapsed) ' . Saving tmp result..']);
        params.network=ncontroller.serializeNetwork;
        params.stats=ncontroller.serializePlotData;
        save(['tmp_' resultFileName '_' host],'params');
        if switchRw
            if params.rwpunish
                disp('switching to reward attraction');
                ncontroller.setPunish(false);
                params.rwpunish=false;
            else
                disp('switching to reward avoidance');
                ncontroller.setPunish(true);
                params.rwpunish=true;
            end
        end
    end
    %% end learning condition5
    if wDiffTurnLeft>3.6 && wDiffTurnRight > 3.6
        disp(['Training finished - Stop condition reached.']);
        break;
    end
    
end
telapsed=toc;
%% end MAIN LOOP
disp(['Training finished - simulation time:'  num2str(dsec*10) 'sec CPU time:' num2str(telapsed) 'sec.']);
disp('Serializing network ... ');
params.network=ncontroller.serializeNetwork;
params.stats=ncontroller.serializePlotData;

save(['trainRobot_' resultFileName '_' host ],'params');
disp('Finished!');

    function Current = calcExternalInputCurrent
        Current=zeros(N,1);
        if (SCDist<SLDist || SCDist<SRDist)   % if either not null and greater than middle - otherwise drive straight
            if SLDist>SRDist
                if cDir==dir.right
                    overshootRight=overshootRight+1;
                end
                Current(MR)=poissrnd(trainMotorCurrentRight,motorSize,1);
            else
                if cDir==dir.left
                    overshootLeft=overshootLeft+1;
                end
                Current(ML)=poissrnd(trainMotorCurrentLeft,motorSize,1);
            end
        end
        if SLDist~=0
            %Current(SL)=poissrnd( (1+SLDist)^params.Robot.FI,sensorSize,1);
            Current(SL)=poissrnd( (SLDist)*params.Robot.FI,sensorSize,1);
        end
        if SRDist~=0
            %IExt(SR)=poissrnd( (1+SRDist)^params.Robot.FI,sensorSize,1);
            Current(SR)=poissrnd( (SRDist)*params.Robot.FI,sensorSize,1);
        end
        Current([ML MR])=Current([ML MR])+poissrnd(extMotorCurrent,motorSize*2,1);
        if extSensorCurrent~=0
            Current([SL SR])=Current([SL SR])+poissrnd(extSensorCurrent,sensorSize*2,1);
        end
    end

    function adjustTrainingCurrent
        if missedCountLeft >1
            disp('intervene: not enough left');
            missedCountLeft=1;missedCountRight=missedCountRight*1.1;
        end
        if missedCountRight>1
            disp('intervene here not enough right');
            missedCountRight=1;trainMotorCurrentLeft=trainMotorCurrentLeft*1.1;
        end
        if overshootLeft>1
            disp('intervene: too much left');
            overshootLeft=1;trainMotorCurrentRight=trainMotorCurrentRight*0.9;
        end
        if overshootRight>1
            disp('intervene: too much right');
            overshootRight=1;trainMotorCurrentLeft=trainMotorCurrentLeft*0.9;
        end
    end

    function Ob=createObstacle(trainMode)
        Ob=[];
        if trainMode==dir.straight || trainMode==dir.straight2
            return;
        end
        
        
        rState = mrobot.getState();
        x= rState(1);
        y= rState(2);
        th= rState(3);
        angle=(45*pi)/180;
        dist=18; % shouldnt bee too far for training: short dist between reward and rewarding event. must reward turn
        
        if trainMode==dir.left
            orientation=th+angle;
        elseif trainMode==dir.right
            orientation=th-angle;
        end
        Ob.type=1;
        Ob.radius = 3;
        Ob.x = x+cos(orientation)*dist;
        Ob.y = y+sin(orientation)*dist;
        Ob.nearest=false;
        Ob.touched=false;
    end

    function drawTouchedOb
        for i = 1:length(Env.Obs)
            if Env.Obs(i).touched
                set(0,'CurrentFigure',f1);
                drawObstacle(Env.wrapped,Env.Obs(i));
                drawnow;
                return;
            end
        end
    end

    function drawRobot(iterations)
        
        %% init
        set(0,'CurrentFigure',f1);
        cla
        hold on;
        
        rState = mrobot.getState();
        x= rState(1);
        y= rState(2);
        th= rState(3);
        
        %% set boundaries
        curr_xlimit= get(gca,'XLim');
        curr_ylimit= get(gca,'YLim');
        half_xdist= (curr_xlimit(2)-curr_xlimit(1))/2;
        half_ydist= (curr_ylimit(2)-curr_ylimit(1))/2;
        % set(f1,'XLim',[x-half_xdist x+half_xdist])
        % set(f1,'YLim',[y-half_ydist y+half_ydist])
        xlim([x-half_xdist x+half_xdist])
        ylim([y-half_ydist y+half_ydist])
        %% plot obstacles
        for i = 1:length(Env.Obs)
            drawObstacle(Env.wrapped,Env.Obs(i));
            if Env.Obs(i).nearest
                line([x Env.Obs(i).x],[y Env.Obs(i).y]);
            end
        end
        
        %% plot robot position
        plot(mrobot.xTraj(1:iterations),mrobot.yTraj(1:iterations),'.')
        
        %% plot sensor ranges
        plot_arc(th+params.Robot.sonarStartAngle,th+pi/2,x,y,params.Robot.sonarRange);
        plot_arc(th-params.Robot.sonarStartAngle,th-pi/2,x,y,params.Robot.sonarRange);
        
        
        hold off;
        drawnow;
        
    end

end
