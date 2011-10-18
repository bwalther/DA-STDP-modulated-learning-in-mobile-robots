function plotLearningStats(params)


N=length(params.network.a);
sm=params.network.sm;
s=params.stats.s;
firings=params.stats.lastfirings;
weightMean=params.stats.weightMean;
dsec=size(weightMean,1);
HZPERGRP=params.stats.HZPERGRP;
MIdx=100:900;
    

subplot(3,2,1)
    Hz=length(firings(:,2))/100/N*1000;
    plot(firings(:,1),firings(:,2),'.');
    title(['Firings: ' num2str(Hz) 'Hz']);
    axis([0 100 0 N]);


dsec=size(weightMean,1);

subplot(3,2,2)
hist(s(find(s(MIdx)>0)),sm*(0.01:0.01:1)); % only excitatory synapses
title('Weights M-*');

subplot(3,2,3);
plot(1:dsec,weightMean(:,1:2) );
legend('SL-ML','SL-MR','Location','NorthWest');
title('Mean Weights SL-*');
ylabel('weight');

subplot(3,2,4);
plot(1:dsec,weightMean(:,3:4) );
legend('SR-ML','SR-MR','Location','NorthWest');
title('Mean Weights SR-*');
ylabel('weight');

subplot(3,2,5:6);
plot(1:dsec,HZPERGRP,'-');
title('Firing rates of different layers');
ylabel('Hz');
xlabel('dsec');
legend('SL','SR','ML','MR','IL','IR','Location','NorthWest');


drawnow;