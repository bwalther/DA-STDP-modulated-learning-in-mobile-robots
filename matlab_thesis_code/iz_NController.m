classdef iz_NController<handle
    
    properties(GetAccess= 'protected', SetAccess= 'private')
        
        % gui and logging
        f1Contr=0;guiOn=false;logtrace=false;
        % config
        ads=false;stdpsym=false;stdpvals=[];STDPwrapover=2;DArw=0.5;
        
        % network
        M=200;                 % number of synapses per neuron
        D=1;                   % maximal conduction delay
        Ns1=50; Ns2=50;
        Ne1=400; Ne2=400;
        Ni1=100; Ni2=100;
        Ns=0;Ne=0;Ni=0;N=0;
        
        % indices of layers
        SL=[]; SR=[]; ML=[]; MR=[]; IL=[]; IR=[];

        % connectivity variables
        delays=[]; post=[]; pre=[]; aux=[];
        % connection indices 
        cSLML=[]; cSLMR=[]; cSRML=[]; cSRMR =[];
        
        % dopamine increases for reward and punishments
        attrDA=0.5;punDA=-0.2;
        
        
         % maximal synaptic strength % synaptic weights % their derivatives
        sm=4;  s=[]; sd=[];LTDstats=[];LTPstats=[];
        % sum of weights sensor to motor layer, used for synaptic scaling:
        smTM=0;
        
        % stdp traces for each synapse % level of dopamine above the baseline
        STDP = []; DA=0;
        
        % voltage, refractory variable, neuron firings (reset after each
        % step)
        v = [];  u = []; firings=[];
        % iz parameters
        a=[];  b=[]; c=[]; d=[];
        
        % stats
        lastfirings=[];
        weightMean=[];
        daStats=[];
        HZPERGRP=[];        
    end
    methods (Access='private')
        
        function buildProperties(obj)
            obj.Ns=obj.Ns1+obj.Ns2;
            obj.Ne=obj.Ne1+obj.Ne2;
            obj.Ni=obj.Ni1+obj.Ni2;
            obj.N=obj.Ns+obj.Ne+obj.Ni;
            
            % config for heterogenity; goal: stability, no overtraining (too much spikings in
            % Motor layer after learning,  WTA
            re=rand(obj.Ns+obj.Ne,1); ri=rand(obj.Ni,1);
            obj.a=[0.02*ones(obj.Ns,1); 0.2*ones(obj.Ne,1); 0.1*ones(obj.Ni,1) ]; % 0.001 (for iz Grp)
            obj.b=[0.2*ones(obj.Ns,1); 0.2*ones(obj.Ne,1);  0.25-0.05*ri ]; % 0.25-0.05*ri, 0.237*
            obj.c=[-65+15*re.^2; -65*ones(obj.Ni,1)]; % -65+15*re.^2
            obj.d=[   8*ones(obj.Ns,1);   8*ones(obj.Ne,1);   2*ones(obj.Ni,1) ];
            
            % indices of layers
            obj.SL=1:obj.Ns1;
            obj.SR=obj.Ns1+1:obj.Ns;
            obj.ML=obj.Ns+1:obj.Ns+obj.Ne1;
            obj.MR=obj.Ns+obj.Ne1+1:obj.Ns+obj.Ne;
            obj.IL=obj.Ns+obj.Ne+1:obj.Ns+obj.Ne+obj.Ni1;
            obj.IR=obj.Ns+obj.Ne+obj.Ni1+1:obj.N;
            
            % 50% (M*0.25=100 connections per neuron) connectivity between Motor and Inhibitory layer:
            sex=rand(obj.Ne,obj.M).*(rand(obj.Ne,obj.M)>0.25);
            obj.s=[rand(obj.Ns,obj.M);sex;-1*rand(obj.Ni,obj.M)];
            obj.smTM=sum(sum(obj.s(1:obj.Ns,:)));
            obj.sd=zeros(obj.N,obj.M);
            obj.LTDstats=zeros(obj.N,obj.M);
            obj.LTPstats=zeros(obj.N,obj.M);
            % rows: neuron numbers, col: time 1-1000, value: decaying stdp over time
            obj.STDP = zeros(obj.N,100+obj.STDPwrapover);
            obj.v = -65*ones(obj.N,1);                   
            obj.u = 0.2.*obj.v;         
            obj.firings=[-obj.D 0];
 
        end
        
        function initConnectivity(obj)
            mRec= ceil(0.2/obj.M); % recurrent proecting to own
            mRec=20; % alternatively fixed value to have some drift
            %mRec=0; % no recurrent connections
            obj.post=[randi([obj.ML(1), obj.MR(end)],[length(obj.SL), obj.M]); % SL->M
                randi([obj.ML(1), obj.MR(end)],[length(obj.SR), obj.M]); % SR->M
                
                randi([obj.ML(1), obj.ML(end)],[length(obj.ML), mRec]), randi([obj.IR(1), obj.IR(end)],[length(obj.ML), obj.M-mRec]);    % ML->ML (for some self sustained activity) & ML->IR (WTA)
                randi([obj.MR(1), obj.MR(end)],[length(obj.MR), mRec]), randi([obj.IL(1), obj.IL(end)],[length(obj.MR), obj.M-mRec]); % MR->MR (") & MR->IL (WTA)
                
                randi([obj.ML(1), obj.ML(end)],[length(obj.IL), obj.M/2]), randi([obj.IR(1), obj.IR(end)],[length(obj.IL), obj.M/2]) ;% IL ->ML & IL -> IR (WTA)
                randi([obj.MR(1), obj.MR(end)],[length(obj.IR), obj.M/2]), randi([obj.IL(1), obj.IL(end)],[length(obj.IR), obj.M/2]) ];% IR ->MR & IR -> IL (WTA)
            
            % PRE: TO-FROM (~100 (where s>0) )
            for i=1:obj.N
                if i<=obj.Ns+obj.Ne
                    for j=1:obj.D
                        obj.delays{i,j}=obj.M/obj.D*(j-1)+(1:obj.M/obj.D);
                    end;
                else
                    obj.delays{i,1}=1:obj.M;
                end;
                obj.pre{i}=find(obj.post==i&obj.s>0);
                % used for STDP indexing matching to pre
                obj.aux{i}=obj.N*(obj.D-1-ceil(ceil(obj.pre{i}/obj.N)/(obj.M/obj.D)))+1+mod(obj.pre{i}-1,obj.N); 
            end;
            
            % Indexes between Sensor and Excitatory layers (indexing post variable)
            matSize=size(obj.post);
            for z=1:obj.Ns1
                for z2=1:obj.M
                    for z3=1:obj.Ne1
                        if obj.post(obj.SL(z),z2)==obj.ML(z3)
                            obj.cSLML=[obj.cSLML,sub2ind(matSize,obj.SL(z),z2)];
                        end
                        if obj.post(obj.SL(z),z2)==obj.MR(z3)
                            obj.cSLMR=[obj.cSLMR,sub2ind(matSize,obj.SL(z),z2)];
                        end
                        if obj.post(obj.SR(z),z2)==obj.ML(z3)
                            obj.cSRML=[obj.cSRML,sub2ind(matSize,obj.SR(z),z2)];
                        end
                        if obj.post(obj.SR(z),z2)==obj.MR(z3)
                            obj.cSRMR=[obj.cSRMR,sub2ind(matSize,obj.SR(z),z2)];
                        end
                    end
                end
            end
            
        end
        

        function initFigures(obj)
            obj.f1Contr=figure(10);
        end
    end
    
    methods
        function assignNetwork(obj,network)
            obj.M=network.M; 
            obj.D=network.D; 
            obj.Ns1=network.Ns1; obj.Ns2=network.Ns2;
            obj.Ne1=network.Ne1; obj.Ne2=network.Ne2;
            obj.Ni1=network.Ni1; obj.Ni2=network.Ni2;
            obj.Ns=obj.Ns1+obj.Ns2;
            obj.Ne=obj.Ne1+obj.Ne2;
            obj.Ni=obj.Ni1+obj.Ni2;
            obj.N=obj.Ns+obj.Ne+obj.Ni;
            
            obj.a=network.a;
            obj.b=network.b;
            obj.c=network.c;
            obj.d=network.d;
            
            % indices of layers
            obj.SL=1:obj.Ns1;
            obj.SR=obj.Ns1+1:obj.Ns;
            obj.ML=obj.Ns+1:obj.Ns+obj.Ne1;
            obj.MR=obj.Ns+obj.Ne1+1:obj.Ns+obj.Ne;
            obj.IL=obj.Ns+obj.Ne+1:obj.Ns+obj.Ne+obj.Ni1;
            obj.IR=obj.Ns+obj.Ne+obj.Ni1+1:obj.N;
            
            obj.s=network.s;
            obj.smTM=sum(sum(obj.s(1:obj.Ns,:)));
            obj.sm=network.sm;
            obj.sd=zeros(obj.N,obj.M);
            obj.LTDstats=zeros(obj.N,obj.M);
            obj.LTPstats=zeros(obj.N,obj.M);
            
            obj.STDP = zeros(obj.N,100+obj.STDPwrapover);
            obj.v = -65*ones(obj.N,1); 
            obj.u = 0.2.*obj.v; 
            obj.firings=[-obj.D 0];
            
            % connectivity
            obj.post=network.post;
            obj.pre=network.pre;
            obj.aux=network.aux;
            obj.delays=network.delays;
            obj.cSLML=network.cSLML;
            obj.cSLMR=network.cSLMR;
            obj.cSRML=network.cSRML;
            obj.cSRMR=network.cSRMR;
            
        end
        function setPunish(obj,doPunish)
           if doPunish
               obj.DArw=obj.punDA;
           else
               obj.DArw=obj.attrDA;
           end
        end
        function obj=iz_NController(params)
            obj.logtrace=params.logtrace;
            if params.rwpunish
                obj.DArw=obj.punDA;
            else
                obj.DArw=obj.attrDA;
            end
            obj.ads=params.ads;
            obj.stdpsym=params.stdpsym;
            if obj.stdpsym
                obj.stdpvals=symSTDPValues(params.STDP.sym);
                obj.STDPwrapover=length(obj.stdpvals);
            else
                obj.STDPwrapover=1+obj.D;
            end
            
            if params.assignNetwork
                assignNetwork(obj,params.network);
            else
                buildProperties(obj);
                initConnectivity(obj);
            end
            obj.guiOn=params.guiOn;
            if params.guiOn
                initFigures(obj);
            end
        end
        
        function [mSL mSR mML mMR mIL mIR mN]=topologyInfo(obj)
            mSL=obj.SL;
            mSR=obj.SR;
            mML=obj.ML;
            mMR=obj.MR;
            mIL=obj.IL;
            mIR=obj.IR;
            mN=obj.N;
        end
        function data=serializePlotData(obj)
                data.weightMean=obj.weightMean;
                data.weightMeanSnapshot=obj.getWeightMeans;
                data.HZPERGRP=obj.HZPERGRP;
                data.s=obj.s;
                data.lastfirings=obj.lastfirings;
        end
        function wmeans=getWeightMeans(obj)
            wmeans(1)=mean(obj.s(obj.cSLML));            
            wmeans(3)=mean(obj.s(obj.cSLMR));
            wmeans(5)=mean(obj.s(obj.cSRML));
            wmeans(6)=mean(obj.s(obj.cSRMR));
        end
        function [network]=serializeNetwork(obj)
            network.M=obj.M; 
            network.D=obj.D;
            network.Ns1=obj.Ns1; network.Ns2=obj.Ns2;
            network.Ne1=obj.Ne1; network.Ne2=obj.Ne2;
            network.Ni1=obj.Ni1; network.Ni2=obj.Ni2;
            
            network.a=obj.a;
            network.b=obj.b;
            network.c=obj.c;
            network.d=obj.d;
            
            network.s=obj.s;
            network.smTM=obj.smTM;
            network.sm=obj.sm;
            
            % connectivity
            network.post=obj.post;
            network.pre=obj.pre;
            network.aux=obj.aux;
            network.delays=obj.delays;
            network.cSLML=obj.cSLML;
            network.cSLMR=obj.cSLMR;
            network.cSRML=obj.cSRML;
            network.cSRMR=obj.cSRMR;
        end
        function [retHZPERGROUP wDiffTurnLeft wDiffTurnRight]=simulationStep(obj,dsec,rw,IExt)
            for t=1:100
                I=13*(rand(obj.N,1)-0.5)+IExt;               
                fired = find(obj.v>=30);                % indices of fired neurons
                obj.v(fired)=obj.c(fired);
                obj.u(fired)=obj.u(fired)+obj.d(fired);
                if obj.stdpsym
                    obj.STDP(fired,t+obj.D:t+obj.D+50-1)=repmat(obj.stdpvals,length(fired),1);
                else
                    obj.STDP(fired,t+obj.D)=0.005; % Note a incoming spike tin the future t+D
                end
                for k=1:length(fired)
                    obj.sd(obj.pre{fired(k)})=obj.sd(obj.pre{fired(k)}) +obj.STDP(obj.N*t+obj.aux{fired(k)}); % weight increase                      
                end;
                obj.firings=[obj.firings;t*ones(length(fired),1),fired];
                k=size(obj.firings,1);
                while obj.firings(k,1)>t-obj.D
                    del=obj.delays{obj.firings(k,2),t-obj.firings(k,1)+1};
                    ind = obj.post(obj.firings(k,2),del);
                    I(ind)=I(ind)+obj.s(obj.firings(k,2), del)';
                    if obj.stdpsym
                        obj.sd(obj.firings(k,2),del)=obj.sd(obj.firings(k,2),del) +obj.STDP(ind,t+obj.D)';
                    else
                        obj.sd(obj.firings(k,2),del)=obj.sd(obj.firings(k,2),del) -1.05*obj.STDP(ind,t+obj.D)'; % weight decrease
                    end
                    k=k-1;
                end;
                
                dt=0.25;
                k=1/dt;
                stepfired=[];
                for ei=1:k
                    obj.v=obj.v+dt*((0.04*obj.v+5).*obj.v+140-obj.u+I);    
                    obj.u=obj.u+dt*(obj.a.*(obj.b.*obj.v-obj.u));            
                    stepfired = [stepfired;find(obj.v>=30)];                
                    obj.v(stepfired)=31;
                end
                if ~obj.stdpsym
                    obj.STDP(:,t+obj.D+1)=0.95*obj.STDP(:,t+obj.D); 
                end
                obj.DA=obj.DA*0.995;
                if (mod(t,10)==0)
                    obj.s(1:obj.Ns+obj.Ne,:)=max(0,min(obj.sm,obj.s(1:obj.Ns+obj.Ne,:)+(obj.DA+0.002)*obj.sd(1:obj.Ns+obj.Ne,:)));
                     if obj.ads
                         obj.s(1:obj.Ns,:)=obj.s(1:obj.Ns,:).*obj.smTM/sum(sum(obj.s(1:obj.Ns,:)));
                     end
                     if obj.guiOn && obj.logtrace && obj.DA > 0.2 
                         tmpL=mean(obj.sd(obj.cSLMR));
                         tmpLwrong=mean(obj.sd(obj.cSLML));
                         tmpR=mean(obj.sd(obj.cSRML));
                         tmpRwrong=mean(obj.sd(obj.cSRML));
                         if tmpL<tmpLwrong
                             disp(['wchange in wrong direction SL:' num2str((obj.DA)*tmpLwrong)]);
                         elseif tmpL>0
                             disp(['CORRECT wchagne SL:' num2str((obj.DA)*tmpL)]);
                         end
                         if tmpR<tmpRwrong
                             disp(['wchange in wrong direction SR:' num2str((obj.DA)*tmpRwrong)]);
                         elseif tmpR>0
                             disp(['CORRECT wchange SR:' num2str((obj.DA)*tmpR)]);
                         end
                     end
                    obj.sd=0.99*obj.sd; % eligibility trace
                end;
                
                if any(rw==100*dsec+t)
                    obj.DA=obj.DA+obj.DArw;
                end
                
            end;
            
            CURFIRINGS=sort(obj.firings(:,2));
            % -1 since we add this in the beginning of the iteration
            obj.HZPERGRP(dsec,1)=(sum(CURFIRINGS<(obj.SL(end)+1))-1)/obj.Ns1*10;
            obj.HZPERGRP(dsec,2)=sum(CURFIRINGS>obj.SL(end) & CURFIRINGS<obj.SR(end)+1)/obj.Ns2*10;
            obj.HZPERGRP(dsec,3)=sum(CURFIRINGS>obj.SR(end) & CURFIRINGS<obj.ML(end)+1)/obj.Ne1*10;
            obj.HZPERGRP(dsec,4)=sum(CURFIRINGS>obj.ML(end) & CURFIRINGS<obj.MR(end)+1)/obj.Ne2*10;
            obj.HZPERGRP(dsec,5)=sum(CURFIRINGS>obj.MR(end) & CURFIRINGS<obj.IL(end)+1)/obj.Ni1*10;
            obj.HZPERGRP(dsec,6)=sum(CURFIRINGS>obj.IL(end) & CURFIRINGS<obj.IR(end)+1)/obj.Ni2*10;
            retHZPERGROUP=obj.HZPERGRP;
            
            obj.weightMean(dsec,1)=mean(obj.s(obj.cSLML));
            obj.weightMean(dsec,2)=mean(obj.s(obj.cSLMR));
            obj.weightMean(dsec,3)=mean(obj.s(obj.cSRML));
            obj.weightMean(dsec,4)=mean(obj.s(obj.cSRMR));
            obj.daStats(dsec)=obj.DA;
            wDiffTurnLeft=obj.weightMean(dsec,2)-obj.weightMean(dsec,1);
            wDiffTurnRight=obj.weightMean(dsec,3)-obj.weightMean(dsec,4);
            
            if obj.guiOn
                
                set(0,'CurrentFigure',obj.f1Contr);
                subplot(3,2,1)
                Hz=length(obj.firings(:,2))/100/obj.N*1000;
                plot(obj.firings(:,1),obj.firings(:,2),'.');
                title(['Firings: ' num2str(Hz) 'Hz']);
                axis([0 100 0 obj.N]);
                
                subplot(3,2,2)
                 hist(obj.s(find(obj.s>0)),obj.sm*(0.01:0.01:1)); % only excitatory synapses
                %tmpsyn=obj.s([obj.ML, obj.MR],:);
                %hist(tmpsyn(:),obj.sm*(0.01:0.01:1)); % only motor to inh

%                 subplot(3,2,2);
%                 plot(1:dsec,obj.daStats );
%                 title('Dopamine Levels');
%                 ylabel('DA');
%                  
                subplot(3,2,3);
                plot(1:dsec,obj.weightMean(:,1:2) );
                legend('SL-ML','SL-MR','Location','NorthWest');
                title('Mean Weights SL-*');
                ylabel('weight');
                
                subplot(3,2,4);
                plot(1:dsec,obj.weightMean(:,3:4) );
                legend('SR-ML','SR-MR','Location','NorthWest');
                title('Mean Weights SR-*');
                ylabel('weight');
                
                subplot(3,2,5:6);
                plot(1:dsec,obj.HZPERGRP,'-');
                title('Firing rates of different layers');
                ylabel('Hz');
                xlabel('dsec');
                legend('SL','SR','ML','MR','IL','IR','Location','NorthWest');
                
                
                drawnow;
                % ---- end plot ------
            end
            
            obj.STDP(:,1:obj.STDPwrapover)=obj.STDP(:,101:100+obj.STDPwrapover);
            ind = find(obj.firings(:,1) > 101-obj.D);
            obj.lastfirings=obj.firings;
            obj.firings=[-obj.D 0;obj.firings(ind,1)-100,obj.firings(ind,2)];
            
            
            
        end
        
        
    end
    
    
end