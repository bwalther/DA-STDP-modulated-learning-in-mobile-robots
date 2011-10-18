%load('2400paramns','paramns');
ns=params.network.s;


subplot(3,2,1)
hist(ns(params.network.cSLML),4*(0.01:0.01:1))
title('SL-ML')
xlim([0 4]);

subplot(3,2,2)
hist(ns(params.network.cSRML),4*(0.01:0.01:1))
title('SR-ML')
xlim([0 4]);

subplot(3,2,3)
hist(ns(params.network.cSLMR),4*(0.01:0.01:1))
title('SL-MR')
xlim([0 4]);

subplot(3,2,4)
hist(ns(params.network.cSRMR),4*(0.01:0.01:1))
title('SR-MR')
xlim([0 4]);


subplot(3,2,5)
tmp=ns(200:900,:);
hist(tmp(:),4*(0.01:0.01:1))
title('M-*')
xlim([0 4]);


subplot(3,2,6)
%hist(ns(:),4*(0.01:0.01:1));
hist(ns(:),4*(0.01:0.01:1));
title('Network')
xlim([0 4]);