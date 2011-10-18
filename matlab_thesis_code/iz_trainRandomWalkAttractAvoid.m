function iz_trainRandomWalkAttractAvoid(curiosity,distLevel,resultfileName,network)


params.assignNetwork=false;
if nargin ==4
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

curiosityActive=false;


% shouldnt bee too much for training: short dist between reward and rewarding event. must reward turn
rw=[];pun=[];
slidingRmaxWindow=50;rmaxIdx=0;
MOVRmax=repmat(params.Robot.Rmax,slidingRmaxWindow,1);
firingsHist=[];firingHistTscale=0;
params.runs=1;

%% init robot environment
mrobot=Robot(params.Robot);
Env.xmax = 100;
Env.ymax = 100;
Env.nObjects=10;% 10, 5 the more object the more confused robot can get if 2 are in its vision. no clear turning patterns
Env.wrapped=true;
Env.Obs=createObstacles;

%% init test controls
rw=[];rwstats=0;pun=[];punstats=0;


%% init neuron controller
ncontroller.assignProperties(params.network);
[GSL BSL GSR BSR ML MR IL IR N]=ncontroller.topologyInfo;
sensorSize=length(GSL);
motorSize=length(ML);


%% init plotting
if params.guiOn
    f1=figure('Name','Random Training Attract & Avoid','NumberTitle','off');
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
        IExt=calcExternalInputCurrent;
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
        mrobot.updatePosition(HzML,HzMR,Env.wrapped,Env.xmax,Env.ymax);
        rState = mrobot.getState();
        %% sensor reading  (must do this before drawing, so to not "draw over" obstacle)
        [SVALUES Env] = readSensorsAttractAvoid(Env,rState(1),rState(2),rState(3),params,sensors);
        if (sum(any(SVALUES(:,1:3)))>0 ) % if collided or eaten

            %% create new obstacle, remove old one
            if sum(any(SVALUES(1,1:3)))>0
                rw=[rw,(dsec+1)*100+1];rwstats=rwstats+1;
                if params.guiOn
                    drawTouchedOb;
                end
                if params.logtrace
                    disp('Object touched, recreated, Reward issued');
                end
            elseif sum(any(SVALUES(2,1:3)))>0
                pun=[pun,(dsec+1)*100+1];punstats=punstats+1;
                if params.guiOn
                    drawTouchedOb;
                end
                if params.logtrace
                    disp('Object touched, recreated, Punishment issued');
                end
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
        if ~mod(dsec,2000)
            disp('2 minute tmp save action');
            params.network=ncontroller.serializeNetwork;
            params.stats=ncontroller.serializePlotData;
            save(['tmp_' resultfileName '_' host ],'params');
        end
        
    end % end simulation
    tElapsed=toc;
%% end MAIN LOOP
disp(['Training finished - simulation time:'  num2str(dsec*10) 'sec CPU time:' num2str(tElapsed) 'sec.' char(10) ... 
    '#rewards:' num2str(rwstats) ' #punishments:' num2str(punstats)]);
disp('Serializing network ... ');
params.network=ncontroller.serializeNetwork;
params.stats=ncontroller.serializePlotData;
save(['trainRobotAttractAvoid_' resultfileName '_' host ],'params');
disp('Finished!');



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
function Current = calcExternalInputCurrent
        Current=zeros(N,1);
        curiosityActive=false;
        Current=randomWalkMotorCurrent(Current,ML,MR,params.logtrace);
        if SVALUES(1,sensors.SC)>SVALUES(1,sensors.SL) && SVALUES(1,sensors.SC)>SVALUES(1,sensors.SR)
            
        else
            if SVALUES(1,sensors.SL)>SVALUES(1,sensors.SC)
                Current(GSL)=poissrnd(min(0.8,SVALUES(1,sensors.SL))*params.Robot.FI,sensorSize,1);
            end
            if SVALUES(1,sensors.SR)>SVALUES(1,sensors.SC)
                Current(GSR)=poissrnd(min(0.8,SVALUES(1,sensors.SR))*params.Robot.FI,sensorSize,1);
            end
        end
        if SVALUES(2,sensors.SL)~=0
            Current(BSL)=poissrnd(SVALUES(2,sensors.SL)*params.Robot.FI,sensorSize,1);
        end
        
        if SVALUES(2,sensors.SR)~=0
            Current(BSR)=poissrnd(SVALUES(2,sensors.SR)*params.Robot.FI,sensorSize,1);
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
                Current(midx)=Current(midx)+poissrnd(SVALUES(2,sensors.SC)*params.Robot.FI,length(midx),1);
            end
        end
        
        if curiosity
            if SVALUES(1,sensors.SL) >distLevel || SVALUES(2,sensors.SR) >distLevel
                curiosityActive=true;
                Current(MR)=poissrnd((SVALUES(1,sensors.SL)-0.5)/2+0.7,motorSize,1);
            end
            if SVALUES(1,sensors.SL) >distLevel || SVALUES(1,sensors.SL) >distLevel
                curiosityActive=true;
                Current(MR)=poissrnd((SVALUES(1,sensors.SL)-0.5)/2+0.7,motorSize,1);
            end
            
        end
        
        
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
            
        rState = mrobot.getState();
        x= rState(1);
        y= rState(2);
        th= rState(3);
        
        set(0,'CurrentFigure',f1);
        cla
        hold on;
        
        %% plot obstacles
        for i = 1:length(Env.Obs)
            drawObstacle(Env.wrapped,Env.Obs(i),Env.xmax,Env.ymax);
            if Env.Obs(i).nearest
                line([x Env.Obs(i).x],[y Env.Obs(i).y]);
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
        if curiosityActive
            title('Curiosity On');
        else
            title('');
        end
        
        hold off;
        drawnow;
        
    end

end