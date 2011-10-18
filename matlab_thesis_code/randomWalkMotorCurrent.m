function IExt=randomWalkMotorCurrent(IExt,ML,MR,logtrace,forceCurve)
% input:  IExt Vector of input current per neuron
%         ML   Indices of left motor neurons
%         MR   Indices of right motor neurons
% output: IExt Updated vector of input currents
persistent countr vL vR waitingPeriod;

mBase=0.3;
if nargin < 5
    forceCurve=false;
end

if isempty(countr)
    countr=0;
    vL=mBase;
    vR=mBase;
    waitingPeriod=10; 
end
countr=countr+1;
%         for lognormal random, mean m=mBase, variance v=0.2;
%             MU = log(M^2 / sqrt(V+M^2))
%             SIGMA = sqrt(log(V/M^2 + 1))
%             random incoming spikes to motor neurons


if forceCurve || mod(countr,waitingPeriod) ==  0
    waitingPeriod=randi([4,14]);          %round(min(lognrnd(0.5816,0.9224)*10,50));
    if forceCurve || rand < 0.6
        if rand > 0.5
            vL=mBase+randi([100 320])/1000;%lognrnd(0.514,0.3201);
            vR=mBase;
        else
            vL=mBase;
            vR=mBase+randi([100 320])/1000;%lognrnd(0.514,0.3201);
        end
    else
        vL=mBase;
        vR=mBase;
    end
end
% DEBUG trace
% disp(['waiting:' num2str(waitingPeriod) ' turningL:' num2str(turningFactorL) ' turingR:' num2str(turningFactorR)]);

% generate a poisson random stream with ?=vL and length ML
if logtrace
    disp([num2str(vL) ' ' num2str(vR) ' wait:' num2str(waitingPeriod)]);
end
IExt(ML)=poissrnd(vL,length(ML),1);
IExt(MR)=poissrnd(vR,length(MR),1);

end
