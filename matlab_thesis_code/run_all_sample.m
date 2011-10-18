% STARTUP FILE % 
% Should be self-explainatory

% ==================================================================== % 
%% 1. benchmark learnt behaviour and random walk
% stats with 1100 network, 100x100 torous:
% a) 15 Objects 
%    attract: ~41/min; avoid: 3/min; rand: 9/min
% b) 10 s food, 10 Objects obstacles: 
%    attractavoid: 32rw/min,1 p/min, rand: 4rw/min, 5p/min
iz_benchmarkRobot('bm_attract',CONNECTBEHAVIOUR.ATTRACT);
iz_benchmarkRobot('bm_avoid',CONNECTBEHAVIOUR.AVOID);
iz_benchmarkRobot('bm_random',CONNECTBEHAVIOUR.RANDOM);
iz_benchmarkRobotAttractAvoid('bm_attractavoid',CONNECTBEHAVIOUR.ATTRACTAVOID);
iz_benchmarkRobotAttractAvoid('bm_attractavoid_rand',CONNECTBEHAVIOUR.RANDOM);
% ==================================================================== % 


% ==================================================================== % 
%% 2. Training 
% Training can be done supervised or unsupervised. 
% Supervised: Single obj. at the time in world. Training current induced to
% approach or avoid object.
% Unsupervised: 15 Obj. in the world. Robot moves with random walk, learns
% coincidentally (no external training current).
% Unsupervised variant: "Curiosity": External training current enforces
% robot to approach object when in certain proximity. 
% 
% Robot can learn either attraction or avoidance of objects or both.
% - The creation of the network goes some time and therefore it is advidable
% to cache the (untrained) network.
% - For maximal performance set guiOn=false and logtrace=false in the iz_*
% files.
% - a training usually goes between 10min - 4h (sim time) depending on
% training technique and behaviour learning.
%
% SUPERVISED
load('templateConfigParamsSNW','network');
iz_trainRobot('trained_1100_network',network);
% alternatively create a new one (this takes some time) by avoiding second
% parameter: iz_trainRobot('trained_1100_network');
load('templateConfigParamsAttrAvoidSNW','network');
% training 2 behaviours
iz_trainRobotAttractAvoid('trained_1100_attrAvoid',network);
%
% UNSUPERVISED
load('templateConfigParamsSNW','network');
iz_trainRobotRandomWalk(false,0,'train_rand',network);
% with curiosity if object very close. Value: 0.8 means: dist obj-robot is
% 20% of solar range.
iz_trainRobotRandomWalk(true,0.8,'train_rand',network);
load('templateConfigParamsAttrAvoidSNW','network');
% training 2 behaviours random walk
iz_trainRandomWalkAttractAvoid(true,0.8,'trained_1100_attrAvoid',network);
% ==================================================================== % 


% ==================================================================== % 
% 3. Plotting 
load('trained_1100_network','params');
plotLearningStats(params);
plotWeightsPerLayer;

load('trained_1100_attrAvoid','params');
plotLearningStatsAttractAvoid(params); % different plotting for 2 behaviours:
plotWeightsPerLayer;
% ==================================================================== %

% ==================================================================== %
% 4. Testing the network
load('data/train_pun_rdwk_SNW_30min','params');
iz_testRobot(params.network,'testPunRandom',true);
load('data/rdwk_attr_curiosity08','params');
iz_testRobot(params.network,'testPunRandom',false);
load('data/trainAttrAvoidRandom','params');
iz_testRobotAttractAvoid('testAttrAvoid4h',params.network);
