function iz_testRobotAttractAvoid(resultFileName,network)
if nargin <1
    error('Specify resultFileName');
end
params.assignNetwork=true;
params.network=network;

extMotorCurrent=0.2;
extSensorCurrent=0.0;
params.logtrace=false;
params.ads=false;
params.stdpsym=false;
params.guiOn=true;
ncontroller=iz_NControllerAttractAvoid(params);


params=configRobot(params);
% params.Robot.FI=12;
params.T=60*60*4; %time in seconds
[res host]=unix('hostname');
host=strtrim(host);

slidingRmaxWindow=50;rmaxIdx=0;
params.runs=1;

% %hardcoding weigths
% params.network.s(params.network.cGSLML)=0.5;
% params.network.s(params.network.cBSLML)=1.8;
% 
% params.network.s(params.network.cGSLMR)=1.8;
% params.network.s(params.network.cBSLMR)=0.5;
% 
% params.network.s(params.network.cGSRML)=1.8;
% params.network.s(params.network.cBSRML)=0.5;
% 
% params.network.s(params.network.cGSRMR)=0.5;
% params.network.s(params.network.cBSRMR)=1.8;

behText='ATTRACT & AVOID';

%% init test controls
rw=[];rwstats=zeros(params.runs,1);pun=[];punstats=zeros(params.runs,1);

%% init robot environment
mrobot=Robot(params.Robot);
Env.xmax = 100;
Env.ymax = 100;
Env.nObjects=10;% 10, 5 the more object the more confused robot can get if 2 are in its vision. no clear turning patterns
Env.wrapped=true;
Env.Obs=createObstacles;

%% init neuron controller
ncontroller.assignProperties(params.network);
[GSL BSL GSR BSR ML MR IL IR N]=ncontroller.topologyInfo;
sensorSize=length(GSL);
motorSize=length(ML);
MOVRmax=repmat(params.Robot.Rmax,slidingRmaxWindow,1);

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
perIdx=1;
for r=1:params.runs
    tStart=tic;
    for dsec=1:params.T*10
        
        %% calc sensor spike source current
        IExt=zeros(N,1);
        if SVALUES(1,sensors.SL)>SVALUES(1,sensors.SC)
            IExt(GSL)=poissrnd(min(0.8,SVALUES(1,sensors.SL))*params.Robot.FI,sensorSize,1);
        end
        if SVALUES(2,sensors.SL)~=0
            IExt(BSL)=poissrnd(SVALUES(2,sensors.SL)*params.Robot.FI,sensorSize,1);
        end
        if SVALUES(1,sensors.SR)>SVALUES(1,sensors.SC)
            IExt(GSR)=poissrnd(min(0.8,SVALUES(1,sensors.SR))*params.Robot.FI,sensorSize,1);
        end
        if SVALUES(2,sensors.SR)~=0
            IExt(BSR)=poissrnd(SVALUES(2,sensors.SR)*params.Robot.FI,sensorSize,1);
        end
        % avoid in the front!
        if SVALUES(2,sensors.SC)~=0
            stimulusIdx=[];
            if SVALUES(2,sensors.SL)>SVALUES(2,sensors.SR)
                 stimulusIdx=BSL;
            elseif SVALUES(2,sensors.SL)<SVALUES(2,sensors.SR)
                stimulusIdx=BSR;
            else
                stimulusIdx=[BSL ]; % random go left or right.. % hope someone wins with WTA
            end% 
            % no min, emergency turn
            IExt(stimulusIdx)=IExt(stimulusIdx)+poissrnd(SVALUES(2,sensors.SC)*params.Robot.FI,length(stimulusIdx),1);
        end
        IExt([ML MR])=IExt([ML MR])+poissrnd(extMotorCurrent,motorSize*2,1);
        if extSensorCurrent~=0
            IExt([SL SR])=IExt([SL SR])+poissrnd(extSensorCurrent,sensorSize*2,1);
        end
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
        mrobot.updatePosition(HzML,HzMR,Env.wrapped,Env.xmax,Env.ymax);
        rState = mrobot.getState();
        %% sensor reading  (must do this before drawing, so to not "draw over" obstacle)
        [SVALUES Env] = readSensorsAttractAvoid(Env,rState(1),rState(2),rState(3),params,sensors);
        if ( sum(any(SVALUES(:,1:3)))>0 ) % if collided or eaten
            
            %% create new obstacle, remove old one
            if any(SVALUES(1,1:3))
                rwstats(r)=rwstats(r)+1;
                rw=[rw,(dsec+1)*100+1];
                disp('Object touched, recreated, Reward issued');
            else
                punstats(r)=punstats(r)+1;
                pun=[pun,(dsec+1)*100+1];
                disp('Object hit, recreated, Punishment issued');
            end
            if params.guiOn
                drawTouchedOb;
            end
            Env.Obs=recreateTouchedObstacle;
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
            drawRobot(dsec);
        end
        if ~mod(dsec,5000)
            disp('5 minute save action');
            params.network=ncontroller.serializeNetwork;
            params.stats=ncontroller.serializePlotData;
            params.rwPer5min(perIdx)=rwstats(r);
            params.punPer5min(perIdx)=punstats(r);
            perIdx=perIdx+1;
            params.rwstats=rwstats;
            save(['tmpTestAttrAvoid_' resultFileName '_' host '.mat'],'params');
        end
        
    end % end simulation
    tElapsed=toc(tStart);
    disp(['Simulation ' num2str(r) ' Finished. Duration:' num2str(tElapsed) 'sec '....
        '#rewards:' num2str(rwstats(r)) ' #punishments:' num2str(punstats(r)) char(10)  ...
        '--------------------------------------------------------------']);
end % end trial

endTime=toc;
avgTime = endTime/params.runs;
timerList= timerfindall('Name','simStats');
if ~isempty(timerList)
    stop(timerList)
    delete(timerList)
end

%% display stats
disp(['--------------------------------------------------------------' ...
    char(10) ...
    num2str(params.runs) ' runs finished!' ...
   ' Simulation time: ' num2str(params.T) 'sec. CPU Total(time): ' num2str(endTime) 'sec' ...
    ' Avg(time): ' num2str(avgTime) 'sec' char(10)...
    ' #rewards: ' num2str(sum(rwstats)) ' mean per trial:' num2str(mean(rwstats)) ' std' num2str(std(rwstats)) char(10) ... 
    ' #punishments: ' num2str(sum(punstats)) ' mean per trial:' num2str(mean(punstats)) ' std' num2str(std(punstats)) 
    ]);
disp('Serializing network ... ');
params.network=ncontroller.serializeNetwork;
params.stats=ncontroller.serializePlotData;
save(['testAttrAvoid_' resultFileName '_' host '.mat'],'params');
disp('Done');

    function retObs=recreateTouchedObstacle
        obFound=false;
        for j = 1:length(Env.Obs)
            if Env.Obs(j).touched
                Env.Obs(j).x = ceil(rand*Env.xmax);
                Env.Obs(j).y = ceil(rand*Env.ymax);
                Env.Obs(j).nearest=false;
                Env.Obs(j).touched=false;
                obFound=true;
                break;
            end
        end
        if ~obFound
            disp('ERRROR ob not found to recreate');
        end
        retObs=Env.Obs;
    end

    function Obs=createObstacles
        Obs=[];
        for j = 1:Env.nObjects
            
            Ob.x = ceil(rand*Env.xmax);
            Ob.y = ceil(rand*Env.ymax);
            Ob.radius = 3;
            Ob.type=1;
            Ob.nearest=false;
            Ob.touched=false;
            Obs = [Obs ; Ob];
            
            Ob.x = ceil(rand*Env.xmax);
            Ob.y = ceil(rand*Env.ymax);
            Ob.type=2;
            Ob.radius = 3;
            Ob.nearest=false;
            Ob.touched=false;
            
            % Ob.size = MinSize+floor(rand*(MaxSize-MinSize+1));
            Obs = [Obs ; Ob];
            
        end
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
        
        %% init        
        rState = mrobot.getState();
        x= rState(1);
        y= rState(2);
        th= rState(3);
        
        set(0,'CurrentFigure',f1);
        cla
        hold on;
        %% plot obstacles
        for j = 1:length(Env.Obs)
            drawObstacle(Env.wrapped,Env.Obs(j),Env.xmax,Env.ymax);
            if Env.Obs(j).nearest
                line([x Env.Obs(j).x],[y Env.Obs(j).y]);
            end
        end
        
        %% plot robot position
        if iterations > 40
            splot=iterations-40;
            plot(mrobot.xTraj(splot:iterations),mrobot.yTraj(splot:iterations),'.')
        else
            plot(mrobot.xTraj(1:iterations),mrobot.yTraj(1:iterations),'.')
        end
        %% plot sensor ranges
        plot_arc(th+params.Robot.sonarStartAngle,th+pi/2,x,y,params.Robot.sonarRange);
        plot_arc(th-params.Robot.sonarStartAngle,th-pi/2,x,y,params.Robot.sonarRange);
         title([behText ' --- ' ...
               'Score:' num2str(rwstats(r)-2*punstats(r)) ...
               ' #rw: ' num2str(rwstats(r)) ' rw/m:' num2str(floor(600*(rwstats(r)/dsec))) ...
               ' #pun: ' num2str(punstats(r)) ' p/m:' num2str(floor(600*(punstats(r)/dsec))) ...
               ' time:' num2str(floor(dsec/600)) 'min']);
        
        hold off;
        drawnow;
        
    end

end