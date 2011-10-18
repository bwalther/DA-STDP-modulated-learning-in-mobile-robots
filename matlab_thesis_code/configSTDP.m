function params=configSTDP(params)

params.stdpsym=false;

params.STDP.asym.Aplus = 0.1; % values: 0.001, 0.00 (Abott) 0.1 (ms), 1.0 (iz,af)
params.STDP.asym.Aminus = -0.15; % values -0.08 (af) ,-0.8, -0.105, (both ms), -1.5 (iz), -1.1 (bw)
params.STDP.asym.window = 50; % size of STDP window
params.STDP.asym.smin=-1;
params.STDP.asym.smax = 4; % values: 2.0 (ms)
% problem: weights change too fast: setting it to faster exponential decay
params.STDP.asym.Tauminus=20; % values: params.STDP.delta, 5
params.STDP.asym.Tauplus=20;  % values: params.STDP.delta, 5

params.STDP.sym.A = 0.05; % 
params.STDP.sym.window = 50; % size of STDP window
params.STDP.sym.smin=-1;
params.STDP.sym.smax = 4; %
params.STDP.sym.TauA=10; % 
params.STDP.sym.TauB=10;  % 