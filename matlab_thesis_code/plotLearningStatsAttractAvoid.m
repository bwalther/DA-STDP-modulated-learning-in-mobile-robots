function plotNeuralControllerLearningAttractAvoid(params)

N=length(params.network.a);
sm=params.network.sm;
lastfirings=params.stats.lastfirings;
weightMean=params.stats.weightMean;
dsec=size(weightMean,1);
s=params.stats.s;


subplot(3,2,1)
Hz=length(lastfirings(:,2))/100/N*1000;
plot(lastfirings(:,1),lastfirings(:,2),'.');
title(['Firings: ' num2str(Hz) 'Hz']);
axis([0 100 0 N]);

%                 set(0,'CurrentFigure',obj.f1Contr);
%                 subplot(3,2,1:2)
%                 Hz=length(obj.firings(:,2))/100/obj.N*1000;
%                 plot(obj.firings(:,1),obj.firings(:,2),'.');
%                 title(['Firings: ' num2str(Hz) 'Hz']);
%                 axis([0 100 0 obj.N]);

%                 set(0,'CurrentFigure',obj.f1Contr);
%                 subplot(3,2,1:2)
%                 Hz=length(obj.firingHist(:,2))/100/obj.N*1000;
%                 plot(obj.firingHist(:,1),obj.firingHist(:,2),'.');
%                 title(['Firings: ' num2str(Hz) 'Hz']);
%                 axis([0 obj.firingHistTscale*100 0 obj.N]);


%                 obj.firingHistTscale=0;
%                 obj.firingHist=[];

subplot(3,2,2)
hist(s(find(s>0)),sm*(0.01:0.01:1)); % only excitatory synapses
title('network');

subplot(3,2,3);
plot(1:dsec,weightMean(:,1:2) );
legend('GSL-ML','GSL-MR','Location','NorthWest');
title('Mean Weights GSL-*');
ylabel('weight');

subplot(3,2,4);
plot(1:dsec,weightMean(:,3:4) );
legend('BSL-ML','BSL-MR','Location','NorthWest');
title('Mean Weights BSL-*');
ylabel('weight');

subplot(3,2,5);
plot(1:dsec,weightMean(:,5:6) );
legend('GSR-ML','GSR-MR','Location','NorthWest');
title('Mean Weights GSR-*');
ylabel('weight');

subplot(3,2,6);
plot(1:dsec,weightMean(:,7:8) );
legend('BSR-ML','BSR-MR','Location','NorthWest');
title('Mean Weights BSR-*');
ylabel('weight');