%% Lead Controller
% With assumption that the values for Pm and wcp are given(Bode Plot)and the process
% transfer function is also given 
% Lead(sys,pm,wcp,PO,ess, Ts,Tr)
% Author: Jenita John
% Files needed: Stepeval.m, Vector.m 
% Inputs G, pm, wcp and the desired values PO,ess%,Tsettle(2%),Trise(100%)
% G: process transfer function 
% pm: Read off the given Bode plot and enter it as second input 
% wcp: Read off the given Bode plot and enter it as third input
% Figure 1 and Figure 2 gives the step response of uncompensated system 
% Figure 3 Compares Gmu(model) with Gcl 
% Figure 4 gives the bode plot of the open loop system 
function[] = Lead(G,pm,wcp,PO,ess,Ts,Tr);      
 C = (0.01)*(pm-5);                         % damping ratio
 sys = G;
 kpos_u = dcgain(G);                        % open loop DC gain of uncompensated system 
 disp(' ')
 disp('Closed Loop Uncompensated transfer function,Gcl(s):');
 Gcl = feedback(sys,1,-1);
 [Gcl]
 disp('Closed Loop Uncompensated transfer function,Gcl(s) -ZPK:');
 Gcl_zpk = zpk(Gcl);
 [Gcl_zpk]
 disp(' ')
 %% Step 1: Calculating the Actual Uncompensated Closed Loop Step Response Specs
 figure(1);
 disp(' ')
 disp('Actual Uncompensated Response Specs:');
 stepinfo(Gcl);                              %%stepeval(Gcl,7);                           % Change t if required 
 disp(' ')
 wn = (((tand(pm))*wcp)/(2*C));             % Calculates wn
 Kdc = dcgain(Gcl);                         % Closed Loop DC gain 
 disp(' ')
 disp('Kdc:');
 disp(Kdc)
 disp(' ')
 num = Kdc*wn*wn;
 den = 2*C*wn;
 sq = wn*wn;
 %% Step 2:  Calculating the Model Uncompensated Closed Loop Step Response Specs
 Gmu = tf(num,[1 den sq]);
 disp(' ')
 disp('Gmu(s)- Model for the Uncompensated System');
 [Gmu]
 disp(' ')
 figure(2);                                   
 disp(' ')
 disp('Model for the Uncompensated Response Specs:');
 stepinfo(Gmu);                         %%stepeval(Gmu,7);                                    % Change t if required 
 disp(' ')
 %% Comparison Graph of both model and actual uncompensated response 
 figure(3);
 step(Gcl,Gmu); 
 %% Compensated System Specs 
 % Using PO calculate new damping ratio
 C2 = (-log(PO/100))/(sqrt((pi*pi)+ (log(PO/100))^2));
 disp(' ')
 disp('Damping Ratio for given PO: ');
 disp(C2)
 disp(' ');
 disp(' Also refer to the POvsDampingRatio Graph')
 % Using Tsettle calculate the wn for compensated  
 wn_c = 4/(C2*Ts);
 disp(' ')
 disp('wn for the Compensated: ');
 disp(wn_c)
 disp(' ');
 % Compensated Phase Margin
 pm_c = C2*100;
 disp(' ')
 disp('Phase Margin for the Compensated: ');
 disp(pm_c)
 disp(' ');
 % From wn the wcp was calculated 
 wcp_c = (2*C2*wn_c)/(tand(pm_c));
 disp(' ')
 disp('wcp for the Compensated: ');
 disp(wcp_c)
 disp(' ');
 % Bode plot of the open loop system 
 figure(4);
 margin(G);
 [m,p] = bode(G,wcp_c);
 liftangle = -180 + pm_c - p;
 disp('liftangle:') 
 disp(liftangle)
 disp('|G_open(jwcp)|:') 
 disp(m)
 disp(' ');
 disp('Phase: G_open(jwcp):') 
 disp(p) 
 disp(' ');
 %% Calculating the Compensated Model 
 kdc_c = 1 - (ess/100);
 numc =  kdc_c*wn_c*wn_c;
 denc = 2*C2*wn_c;
 sqc = wn_c*wn_c;
 Gmc = tf(numc,[1 denc sqc]);
 disp('Gmu(s)- Model for the Compensated System');
 [Gmc]
 disp(' ')
 %% Calculating a0,a1 and b1
 kpos_c = (1-(ess/100))/(ess/100);                  % Open loop DC gain of compensated system 
 disp(' ')
 disp('K_{pos(c)}: ');
 disp(kpos_c)
 disp(' ');
 a0 = kpos_c/kpos_u;
 disp(' ')
 disp('a0: ');
 disp(a0)
 disp(' ');
 a1 = (1 -(a0*m*cosd(liftangle)))/(wcp_c*m*sind(liftangle));
 disp(' ')
 disp('a1: ');
 disp(a1)
 disp(' ');
 b1 = ((cosd(liftangle))- (a0*m))/((wcp_c)*(sind(liftangle)));
 disp(' ')
 disp('b1: ');
 disp(b1)
 disp(' ');
 Gcc = tf([a1 a0],[b1 1]);
 disp(' ')
 disp('The Controller Transfer Function is then: ');
 [Gcc] 
 disp(' ');
 G_openc = Gcc*G;
 disp(' ')
 disp('The Compensated Open Loop Function is then: ');
 [G_openc]
 disp(' ');
 figure(5);
 hold on;
 bode(G_openc);
 bode(G);
 hold off;
 Go_cc = feedback(G_openc,1,-1);
 disp(' ')
 disp('The Compensated Closed Loop Function is then: ');
 [Go_cc]
 disp('The Compensated Closed Loop Function is then in zpk: ');
 [zpk(Go_cc)]
 disp(' ');
 %% Step Response Specs of Compensated Function 
 figure(6);
 disp(' ')
 disp('Actual Compensated Response Specs:');
 stepinfo(Go_cc);                       %%stepeval(Go_cc,7);
 disp(' ')
 figure(7);
 disp(' ')
 disp('Model Compensated Response Specs:');
 stepinfo(Gmc);                        %%stepeval(Gmc,7)
 disp(' ')
 %% Comparison between Model and Actual Compensated Response 
 %Gmc_o = Gmc*G;
 %Gmcc = feedback(Gmc_o,1,-1);
 %figure(8);
 %step(Go_cc,Gmc,7)
 %% Compensated vs Uncompensated 
 %figure(9);
 %step(Gmc,Gmu,7)
