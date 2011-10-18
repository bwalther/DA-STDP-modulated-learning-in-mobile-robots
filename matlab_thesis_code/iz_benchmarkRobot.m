function benchmarkRobot(resultFileName,connectBehaviour)


%% config simulation
params.logtrace=false;
params.guiOn=true;
params.connectBehaviour=connectBehaviour;
params.T=60*10*1; %time in seconds
params.runs=1;
params.attrAvoid=false;
rmaxIdx=0;
rmaxfix=false;% switch to keep rmax at tuned level
slidingRmaxWindow=50;
extMotorCurrent=0.2;%0.01 needs to be atleast 0.2, otherwise no turning since spiking too low



%% init test controls
rwstats=zeros(params.runs,1);
[res host]=unix('hostname');
host=strtrim(host);

%% init robot environment
params=configRobot(params);
params.Robot.FI=10;
mrobot=Robot(params.Robot);
Env.xmax = 100;
Env.ymax = 100;
if connectBehaviour==CONNECTBEHAVIOUR.ATTRACTAVOID
    Env.nObjects=10;
else
    Env.nObjects=15;% 10, 5 the more object the more confused robot can get if 2 are in its vision. no clear turning patterns
end
Env.wrapped=true;
Env.Obs=createObstacles;

%% init neuron controller
ncontroller=iz_NControllerNOLEARN(params);
topo=ncontroller.topologyInfo;
SL=topo.SL;SR=topo.SR;
ML=topo.ML;MR=topo.MR;
IL=topo.IL;IR=topo.IR;
N=topo.N;

sensorSize=length(SL);
motorSize=length(ML);
MOVRmax=repmat(params.Robot.Rmax,slidingRmaxWindow,1);
firingsHist=[];firingHistTscale=0;

%% init plotting
if params.guiOn
    f10=figure(10);
    f1=figure('Name','Benchmark','NumberTitle','off');
    
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
    rwText='Rewards';
    behText='SEARCH FOOD';
    if connectBehaviour==CONNECTBEHAVIOUR.AVOID
        rwText='Punishments';
        behText='AVOID OBSTACLES';
    end
end
 
%% MAIN LOOP
SLDist=0; SRDist=0;SCDist=0;
tic;
for r=1:params.runs
    tStart=tic;
    for dsec=1:params.T*10
        
        %% calc sensor spike source current
        IExt=zeros(N,1);
        %% random walk if no object close

        if ~(connectBehaviour==CONNECTBEHAVIOUR.AVOID && round(SLDist*100)==round(SRDist*100))
            if SLDist~=0
                IExt(SL)=poissrnd(min(0.8,SLDist)*params.Robot.FI,sensorSize,1);
            end
            if SRDist~=0
                IExt(SR)=poissrnd(min(0.8,SRDist)*params.Robot.FI,sensorSize,1);
            end
        end
        if connectBehaviour==CONNECTBEHAVIOUR.RANDOM
            IExt=IExt+randomWalkMotorCurrent(IExt,ML,MR,params.logtrace);
        else
            % small background noise
            IExt([ML MR])=poissrnd(extMotorCurrent,motorSize*2,1);
        end
        
        %% stimulate network
        [HzPerGroup firings]=ncontroller.simulationStep(dsec,IExt);
        firingsHist=[firingsHist; firings(:,1)+firingHistTscale*100, firings(:,2)];
        firingHistTscale=firingHistTscale+1;
        %% Read out motor firing rate
        HzML = HzPerGroup(dsec,3);
        HzMR = HzPerGroup(dsec,4);
        if params.logtrace
            disp(['Hz: ' num2str(HzPerGroup(dsec,1)) ' -- ' num2str(HzPerGroup(dsec,2)) ' -- ' ...
                num2str(HzML) ' -- ' num2str(HzMR) ' -- ' ...
                num2str(HzPerGroup(dsec,5)) ' -- ' num2str(HzPerGroup(dsec,6)) ]);
        end
        if ~mod(dsec,5000)
            disp('5 minute tmp save action');
            params.network=ncontroller.serializeNetwork;
            params.stats=ncontroller.serializePlotData;
            save(['tmp_' resultFileName '_' host ],'params');
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
            if ~mod(dsec,10)
                updatePlot;
                firingsHist=[];
                firingHistTscale=0;
            end
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
    ]);
disp('Serializing network ... ');
params.network=ncontroller.serializeNetwork;
params.stats=ncontroller.serializePlotData;
params.rwstats=rwstats;
save(['benchmarkRobot_' resultFileName '_' host],'params');
disp('Done');


    function updatePlot
        set(0,'CurrentFigure',f10);
        subplot(2,1,1)
        Hz=length(firingsHist(:,2))/1000/N*1000;
        plot(firingsHist(:,1),firingsHist(:,2),'.');
        title(['Firings: ' num2str(Hz) 'Hz']);
        axis([0 1000 0 N]);
        
        subplot(2,1,2);
        plot(1:dsec,HzPerGroup,'-');
        title('Firing rates of different layers');
        ylabel('Hz');
        xlabel('dsec');
        legend('SL','SR','ML','MR','IL','IR','Location','NorthWest');
        
        
        drawnow;
    end

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

    function Obs=createObstaclesForType(mtype)
        Obs=[];
        for j = 1:Env.nObjects
            Ob.x = ceil(rand*Env.xmax);
            Ob.y = ceil(rand*Env.ymax);
            Ob.radius = 3;
            Ob.type=mtype;
            Ob.nearest=false;
            Ob.touched=false;
            Obs = [Obs ; Ob];            
        end
    end

    function Obs=createObstacles
            if connectBehaviour==CONNECTBEHAVIOUR.ATTRACT || connectBehaviour==CONNECTBEHAVIOUR.RANDOM
                Obs=createObstaclesForType(1);
            elseif connectBehaviour==CONNECTBEHAVIOUR.AVOID
                Obs=createObstaclesForType(2);
            elseif connectBehaviour==CONNECTBEHAVIOUR.ATTRACTAVOID
                Obs=createObstaclesForType(1);
                Obs=[Obs;createObstaclesForType(2)];
            else
                error('behaviour not recognized');
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
        title([behText ' --- ' ...
               '#obj:' num2str(Env.nObjects) ' ' ...
               rwText ': ' num2str(rwstats(r)) ' per min:' num2str(floor(600*(rwstats(r)/dsec))) ...
               ' simtime:' num2str(floor(dsec/600)) 'min']);
        
        hold off;
        drawnow;
        
    end

end