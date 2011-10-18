function iz_trainRobotAttractAvoid(resultfileName,network)

if nargin <1
    error('Specify resultfileName');
end
params.assignNetwork=false;
if nargin ==2
    params.network=network;
    params.assignNetwork=true;
end

params.logtrace=false;
params.rwpunish=true;
params.stdpsym=false;
params.guiOn=true;
params.ads=false;
ncontroller=iz_NControllerAttractAvoid(params);

%% config simulation
params=configSTDP(params);
params=configRobot(params);
params.T=60*60; %time in seconds
[res host]=unix('hostname');
host=strtrim(host);

%% init training controls
robotDirections={'left' 'straight' 'right' 'straight2'};
dir = struct(robotDirections{1},1,robotDirections{2},2,robotDirections{3},3,robotDirections{4},4);
cDir=dir.left;
cType=1; % resource type for obj. creation: 1:attr, 2:avoid
trainStepCount=1;
% shouldnt bee too much for training: short dist between reward and rewarding event. must reward turn
maxTrainStep=12;
rw=[];pun=[];
extMotorCurrent=0.2;
extSensorCurrent=0.0;
TRAINCURRENT=[0.8 0.8;0.8 0.8] ;
slidingRmaxWindow=50;rmaxIdx=0;
MOVRmax=repmat(params.Robot.Rmax,slidingRmaxWindow,1);
% missedCountLeft=0;missedCountRight=0;overshootLeft=0;overshootRight=0;
CURRENTCONTROL=zeros(2,4);
cctrl=struct('missedLeft',1,'missedRight',2','overshootLeft',3,'overshootRight',4);

params.runs=1;

%% init robot environment
mrobot=Robot(params.Robot);
Env.xmax = params.Robot.xmax;
Env.ymax = params.Robot.ymax;
Env.wrapped=false;
Env.Obs=createObstacle(cDir);

%% init test controls
rw=[];rwstats=0;pun=[];punstats=0;


%% init neuron controller
ncontroller.assignProperties(params.network);
[GSL BSL GSR BSR ML MR IL IR N]=ncontroller.topologyInfo;
trIdx={GSL,GSR;BSL,BSR};
sensorSize=length(GSL);
motorSize=length(ML);


%% init plotting
if params.guiOn
    f1=figure(1);
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
sensors=struct('TL',1,'TR',2, 'TC', 3, 'SL',4,'SR',5,'SC',6);
SVALUES=zeros(2,6);
    for dsec=1:params.T*10
        
        %% calc sensor spike source current
        IExt=zeros(N,1);
        ls=0;rs=0; % left and right training stimulus
        if SVALUES(cType,sensors.SL)>SVALUES(cType,sensors.SC)
            if cDir==dir.right
                CURRENTCONTROL(cType,cctrl.overshootRight)=CURRENTCONTROL(cType,cctrl.overshootRight)+1;
            end
            rs=rs+SVALUES(cType,sensors.SL);
            IExt(trIdx{cType,1})=poissrnd(min(0.8,SVALUES(cType,sensors.SL))*params.Robot.FI,sensorSize,1);
        end
        if SVALUES(cType,sensors.SR)>SVALUES(cType,sensors.SC)
            if cDir==dir.left
                CURRENTCONTROL(cType,cctrl.overshootLeft)=CURRENTCONTROL(cType,cctrl.overshootLeft)+1;
            end
            ls=ls+SVALUES(cType,sensors.SR);
            IExt(trIdx{cType,2})=poissrnd(min(0.8,SVALUES(cType,sensors.SR))*params.Robot.FI,sensorSize,1);
        end
        
        if ls>rs  %% simple majority, 
            IExt(ML)=poissrnd(TRAINCURRENT(cType,1),motorSize,1);  % (0.7) 0.35 with base motor current, 1.7 without
        elseif rs>ls
            IExt(MR)=poissrnd(TRAINCURRENT(cType,2),motorSize,1);
        end
        % external background noise
        IExt([ML MR])=IExt([ML MR])+poissrnd(extMotorCurrent,motorSize*2,1);
        if extSensorCurrent~=0
            IExt([SL SR])=IExt([SL SR])+poissrnd(extSensorCurrent,sensorSize*2,1);
        end
        %% stimulate network
        [HzPerGroup ]=ncontroller.simulationStep(dsec,rw,pun,IExt);
        
        %% Read out motor firing rate
        HzML = HzPerGroup(dsec,5);
        HzMR = HzPerGroup(dsec,6);
        if params.logtrace
            disp(['Hz: ' num2str(HzPerGroup(dsec,1)) ' -- ' num2str(HzPerGroup(dsec,2)) ' -- ' ...
                num2str(HzPerGroup(dsec,3)) ' -- ' num2str(HzPerGroup(dsec,4)) ' -- ' ...
                num2str(HzML) ' -- ' num2str(HzMR) ' -- ' ...
                num2str(HzPerGroup(dsec,7)) ' -- ' num2str(HzPerGroup(dsec,8)) ]);
        end
        
        %% drive robot with motor neuron output
        mrobot.updatePosition(HzML,HzMR,Env.wrapped,false);
        trainStepCount=trainStepCount+1;
        rState = mrobot.getState();
        %% sensor reading  (must do this before drawing, so to not "draw over" obstacle)
        [SVALUES Env] = readSensorsAttractAvoid(Env,rState(1),rState(2),rState(3),params,sensors);
        if (trainStepCount==maxTrainStep || sum(any(SVALUES(:,1:3)))>0 ) % if collided or eaten

            %% create new obstacle, remove old one
            if sum(any(SVALUES(1,1:3)))>0
                rw=[rw,(dsec+1)*100+1];rwstats=rwstats+1;
                if params.guiOn
                    drawTouchedOb;
                end
            elseif sum(any(SVALUES(2,1:3)))>0
                pun=[pun,(dsec+1)*100+1];punstats=punstats+1;
                if params.guiOn
                    drawTouchedOb;
                end
            elseif cDir==dir.left
                CURRENTCONTROL(cType,cctrl.missedCountLeft)=CURRENTCONTROL(cType,cctrl.missedCountLeft)+1;
            elseif cDir==dir.right
                CURRENTCONTROL(cType,cctrl.missedCountRight)=CURRENTCONTROL(cType,cctrl.missedCountRight)+1;
            end
            %% reset training
            if cDir==dir.right
                cType=cType+1;
                if cType > 2
                    cType=1;
                end
            end
            cDir=cDir+1;
            if cDir >length(robotDirections)
                cDir=1;
            end
            trainStepCount=1;
            adjustTrainingCurrent;
            
            Env.Obs=[createObstacle(cDir)];
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
        if ~mod(dsec,2000)
            disp('2 minute tmp save action');
            params.network=ncontroller.serializeNetwork;
            params.stats=ncontroller.serializePlotData;
            save(['tmp_' resultfileName '_' host '.mat'],'params');
        end
        
    end % end simulation
    tElapsed=toc;
%% end MAIN LOOP
disp(['Training finished - simulation time:'  num2str(dsec*10) 'sec CPU time:' num2str(tElapsed) 'sec.' char(10) ... 
    '#rewards:' num2str(rwstats) ' #punishments:' num2str(punstats)]);
disp('Serializing network ... ');
params.network=ncontroller.serializeNetwork;
params.stats=ncontroller.serializePlotData;
save(['trainRobotAttractAvoid_' resultfileName '_' host '.mat'],'params');
disp('Finished!');

    function adjustTrainingCurrent
        if sum(CURRENTCONTROL>1)>0
            for i=1:2
                for j=1:4
                    if CURRENTCONTROL(i,j)>1
                        CURRENTCONTROL(i,j)=1;
                        if j<cctrl.overshootLeft
                            disp('intervene: not enough current');
                            TRAINCURRENT(i,j)=TRAINCURRENT(i,j)*1.1;
                        else
                            disp('intervene: too much current');
                            TRAINCURRENT(i,j)=TRAINCURRENT(i,j)*0.9;
                        end
                    end
                end
            end
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
        
        Ob.radius = 3;
        Ob.type=cType;
        Ob.x = x+cos(orientation)*dist;
        Ob.y = y+sin(orientation)*dist;
        Ob.nearest=false;
        Ob.touched=false;

    end

    function drawTouchedOb
        for j = 1:length(Env.Obs)
            if Env.Obs(j).touched
                set(0,'CurrentFigure',f1);
                drawObstacle(Env.wrapped,Env.Obs(j),Env.xmax,Env.ymax);
                drawnow;
                return;
            end
        end
    end

    function drawRobot(iterations)
        
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