function iz_benchmarkRobotAttractAvoid(resultFileName,connectBehaviour)
if nargin <1
    error('Specify resultFileName');
end

params.assignNetwork=true;
params.attrAvoid=true;
extMotorCurrent=0.2;
extSensorCurrent=0.0;

%% config simulation
params.connectBehaviour=connectBehaviour;
params.logtrace=false;
params.ads=false;
params.stdpsym=false;
params.guiOn=true;
ncontroller=iz_NControllerNOLEARN(params);
params.network=ncontroller.serializeNetwork;

params=configRobot(params);
% params.Robot.FI=12;
params.T=60*30*1; %time in seconds
[res host]=unix('hostname');
host=strtrim(host);
slidingRmaxWindow=50;rmaxIdx=0;
params.runs=10;

%% init test controls
rwstats=zeros(params.runs,1);punstats=zeros(params.runs,1);

%% init robot environment
mrobot=Robot(params.Robot);
Env.xmax = 100;
Env.ymax = 100;
Env.nObjects=10;% 10, 5 the more object the more confused robot can get if 2 are in its vision. no clear turning patterns
Env.wrapped=true;
Env.Obs=createObstacles;

%% init neuron controller
topo=ncontroller.topologyInfo;
GSL=topo.GSL;BSL=topo.BSL;GSR=topo.GSR;BSR=topo.BSR;
ML=topo.ML;MR=topo.MR;
IL=topo.IL;IR=topo.IR;
N=topo.N;

sensorSize=length(GSL);
motorSize=length(ML);
MOVRmax=repmat(params.Robot.Rmax,slidingRmaxWindow,1);
firingsHist=[];firingHistTscale=0;

%% init plotting
if params.guiOn
    f10=figure(10);
    f1=figure('Name','Benchmark Attract & Avoid','NumberTitle','off');
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
    behText='ATTRACT & AVOID';
    if connectBehaviour==CONNECTBEHAVIOUR.RANDOM
        behText='RANDOM';
    end
end
 
%% MAIN LOOP
tic;
sensors=struct('TL',1,'TR',2, 'TC', 3, 'SL',4,'SR',5,'SC',6);
SVALUES=zeros(2,6);
for r=1:params.runs
    tStart=tic;
    for dsec=1:params.T*10
        
        %% calc sensor spike source current
        IExt=zeros(N,1);
        if SVALUES(1,sensors.SC)>SVALUES(1,sensors.SL) && SVALUES(1,sensors.SC)>SVALUES(1,sensors.SR)
            
        else
            if SVALUES(1,sensors.SL)>SVALUES(1,sensors.SC)
                IExt(GSL)=poissrnd(min(0.8,SVALUES(1,sensors.SL))*params.Robot.FI,sensorSize,1);
            end
            if SVALUES(1,sensors.SR)>SVALUES(1,sensors.SC)
                IExt(GSR)=poissrnd(min(0.8,SVALUES(1,sensors.SR))*params.Robot.FI,sensorSize,1);
            end
        end
        if SVALUES(2,sensors.SL)~=0
            IExt(BSL)=poissrnd(SVALUES(2,sensors.SL)*params.Robot.FI,sensorSize,1);
        end
        
        if SVALUES(2,sensors.SR)~=0
            IExt(BSR)=poissrnd(SVALUES(2,sensors.SR)*params.Robot.FI,sensorSize,1);
        end
        
        % avoid in the front!
        if SVALUES(2,sensors.SC)~=0
            stimulusIdx=[];
            if SVALUES(2,sensors.SL)==SVALUES(2,sensors.SR)==0
                % just rand dir, no min, emergency turn
                if rand > 0.5 
                    midx=BSL;
                else
                    midx=BSR;
                end
                IExt(midx)=IExt(midx)+poissrnd(SVALUES(2,sensors.SC)*params.Robot.FI,length(midx),1);
            end
        end
        if connectBehaviour==CONNECTBEHAVIOUR.RANDOM
            IExt=IExt+randomWalkMotorCurrent(IExt,ML,MR,params.logtrace);
        end
        [HzPerGroup firings]=ncontroller.simulationStep(dsec,IExt);
        firingsHist=[firingsHist; firings(:,1)+firingHistTscale*100, firings(:,2)];
        firingHistTscale=firingHistTscale+1;
      
        
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
                if params.logtrace
                disp('Object touched, recreated, Reward issued');
                end
            else
                punstats(r)=punstats(r)+1;
                if params.logtrace
                disp('Object hit, recreated, Punishment issued');
                end
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
            if ~mod(dsec,10)
                updatePlot;
                firingsHist=[];
                firingHistTscale=0;
            end
        end
        if ~mod(dsec,3000)
            disp('5 minute save action');
            params.network=ncontroller.serializeNetwork;
            params.stats=ncontroller.serializePlotData;
            save(['tmp_' resultFileName '_' host ],'params');
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
save(['benchmarkAttrAvoid_' resultFileName '_' host ],'params');
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
        legend('GSL','BSL','GSR','BSR','ML','MR','IL','IR','Location','NorthWest');
        
        
        drawnow;
    end

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
               '#obj:' num2str(Env.nObjects) ' ' ...
               '#rw: ' num2str(rwstats(r)) ' per min:' num2str(floor(600*(rwstats(r)/dsec))) ...
               ' #pun: ' num2str(punstats(r)) ' per min:' num2str(floor(600*(punstats(r)/dsec))) ...
               ' simtime:' num2str(floor(dsec/600)) 'min']);
        
        hold off;
        drawnow;
        
    end

end