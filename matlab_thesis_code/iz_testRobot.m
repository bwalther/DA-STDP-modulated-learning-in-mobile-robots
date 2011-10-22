function iz_testRobot(network,resultfileName,punish)

params.assignNetwork=true;
params.network=network;
params=configSTDP(params);
params=configRobot(params);
params.rwpunish=punish;
params.ads=false;
params.logtrace=false;
params.assignNetwork=true;
params.logtrace=false;
params.guiOn=true;
extMotorCurrent=0.2;%0.01 needs to be atleast 0.2, otherwise no turning since spiking too low


params.T=60*60; %time in seconds
params.runs=1;
rmaxIdx=0;

rmaxfix=false;% switch to keep rmax at tuned level

slidingRmaxWindow=50;

%% init test controls
rw=[];rwstats=zeros(params.runs,1);
[res host]=unix('hostname');
host=strtrim(host);
rwText='Rewards';
if params.rwpunish
    rwText='Punishments';
end

%% init robot environment
mrobot=Robot(params.Robot);
% mrobot.Umax = mrobot.Umin+mrobot.Umin/2;

Env.xmax = 100;
Env.ymax = 100;   
Env.nObjects=15;% 10, 5 the more object the more confused robot can get if 2 are in its vision. no clear turning patterns
Env.wrapped=true;
Env.Obs=createObstacles;

%% hardcoding weights
params.network.s(params.network.cSLML)=0.65;
params.network.s(params.network.cSLMR)=0.7;

params.network.s(params.network.cSRMR)=0.6;
params.network.s(params.network.cSRML)=0.64;

%% init neuron controller
%ncontroller=iz_NControllerTest(params);
ncontroller=iz_NController(params);
[SL SR ML MR IL IR N]=ncontroller.topologyInfo;
sensorSize=length(SL);
motorSize=length(ML);
MOVRmax=repmat(params.Robot.Rmax,slidingRmaxWindow,1);

%% init plotting
if params.guiOn
    f1=figure('Name','Testing');
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
SLDist=0; SRDist=0;SCDist=0;
directedBySensors=true;
tic;
for r=1:params.runs
    tStart=tic;
    for dsec=1:params.T*10
        
        %% calc sensor spike source current
        IExt=zeros(N,1);
        %% random walk if no object close
        if SLDist==0 && SRDist ==0 && SCDist ==0 && rand>0.5
            forceCurve=false;
            if directedBySensors
                forceCurve=true;
                directedBySensors=false;
            end
            IExt=randomWalkMotorCurrent(IExt,ML,MR,params.logtrace,forceCurve);
        else
            directedBySensors=true;
            if SLDist~=0
                IExt(SL)=poissrnd(min(0.8,SLDist)*params.Robot.FI,sensorSize,1);
            end
            if SRDist~=0
                IExt(SR)=poissrnd(min(0.8,SRDist)*params.Robot.FI,sensorSize,1);
            end
            % small background noise
            IExt([ML MR])=poissrnd(extMotorCurrent,motorSize*2,1);
        end
        
        %% stimulate network
        [HzPerGroup wDiffTurnLeft wDiffTurnRight]=ncontroller.simulationStep(dsec,rw,IExt);
        
        %% Read out motor firing rate
        HzML = HzPerGroup(dsec,3);
        HzMR = HzPerGroup(dsec,4);
        if params.logtrace
            disp(['Hz: ' num2str(HzPerGroup(dsec,1)) ' -- ' num2str(HzPerGroup(dsec,2)) ' -- ' ...
                num2str(HzML) ' -- ' num2str(HzMR) ' -- ' ...
                num2str(HzPerGroup(dsec,5)) ' -- ' num2str(HzPerGroup(dsec,6)) ]);
        end
        if ~mod(dsec,1000)
            telapsed=toc;
            tic;
            disp(['Simulated 1 minute (time: ' num2str(telapsed) '). Saving tmp...']);
            params.network=ncontroller.serializeNetwork;
            params.stats=ncontroller.serializePlotData;
            save(['tmp_' resultfileName '_' host ],'params');
        end
    
        %% drive robot with motor neuron output
        mrobot.updatePosition(HzML,HzMR,Env.wrapped,Env.xmax,Env.ymax);
        rState = mrobot.getState();
        %% sensor reading  (must do this before drawing, so to not "draw over" obstacle)
        [TouchL TouchR TouchC SLDist SRDist SCDist Env] = RobotGetSensorsN(Env,rState(1),rState(2),rState(3),params);
        if ( TouchL || TouchR || TouchC ) % if collided or training finished
            
            %% create new obstacle, remove old one
            if TouchL || TouchR || TouchC
                rwstats(r)=rwstats(r)+1;
                rw=[rw,(dsec+1)*100+1];
                if params.guiOn
                    drawTouchedOb;
                end
                if params.logtrace
                disp('Object touched, recreated, Reward issued');
                end
            end
            Env.Obs=recreateTouchedObstacle;   
        end
        if rmaxIdx>slidingRmaxWindow
            rmaxIdx=1;
        else
            rmaxIdx=rmaxIdx+1;
        end
        if ~rmaxfix && ~mod(dsec,100)
            smr=sort(MOVRmax);
            mrobot.Rmax=max(smr(1:floor(0.9*slidingRmaxWindow)))*0.9;
        end
        
        %% plot robot
        if params.guiOn
            drawRobot(dsec);
        end
        
    end % end simulation
    tElapsed=toc(tStart);
    disp(['Simulation ' num2str(r) ' Finished. Duration:' num2str(tElapsed) 'sec '....
        '#rewards:' num2str(rwstats(r)) char(10)  ...
        '--------------------------------------------------------------']);
end % end trial

endTime=toc;
avgTime = endTime/params.runs;

%% display stats
disp(['--------------------------------------------------------------' ...
    char(10) ...
    num2str(params.runs) ' runs finished!' ...
    ' Simulation time: ' num2str(params.T) 'sec. CPU Total(time): ' num2str(endTime) 'sec' ...
    ' Avg(time): ' num2str(avgTime) 'sec' char(10)...
    ' #rewards: ' num2str(sum(rwstats)) ' mean per trial:' num2str(mean(rwstats)) ' std' num2str(std(rwstats)) ...
    char(10) ...
    '--------------------------------------------------------------']);
disp('Serializing network ... ');
params.network=ncontroller.serializeNetwork;
params.stats=ncontroller.serializePlotData;
params.rwstats=rwstats;
save(['testingRobot_' resultfileName '_' host],'params');
disp('Done');

    function retObs=recreateTouchedObstacle
        for j = 1:length(Env.Obs)
            if Env.Obs(j).touched
                Env.Obs(j).x = ceil(rand*Env.xmax);
                Env.Obs(j).y = ceil(rand*Env.ymax);
                Env.Obs(j).nearest=false;
                Env.Obs(j).touched=false;
                break;
            end
        end
        retObs=Env.Obs;
    end

    function Obs=createObstacles
        Obs=[];
        for j = 1:Env.nObjects
            
            Ob.x = ceil(rand*Env.xmax);
            Ob.y = ceil(rand*Env.ymax);
            Ob.radius = 3;
            if params.rwpunish
                Ob.type=2;
            else
                Ob.type=1;
            end
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
        set(0,'CurrentFigure',f1);
        cla
        hold on;
        
        rState = mrobot.getState();
        x= rState(1);
        y= rState(2);
        th= rState(3);
        
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
        title([
            '#obj:' num2str(Env.nObjects) ' ' ...
            rwText ': ' num2str(rwstats(r)) ' per min:' num2str(floor(600*(rwstats(r)/dsec))) ...
            ' simtime:' num2str(floor(dsec/600)) 'min']);
        
        
        hold off;
        drawnow;
        
    end

end