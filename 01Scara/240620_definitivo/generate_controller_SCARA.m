 % Clear workspace, close figures, and clear command window
clear all;
close all;
clc;
% ATTENZIONE
% Ultimo controllore totale generato
% Questo codice non considera gli steps

% Sampling time
Tc = 0.001;

%  ---------------------------CONTROLLORE 1 --------------------------------------
% Kp+Ki/s * 1/(J*s) +1
% J*s^2+Kp*s+Ki
% s^2+Kp/J*s+Ki/J

Kpv1=6766; 
Tiv1=0.1; 
Kiv1=Kpv1/Tiv1;

Tdv1 = 0.0;
Kdv1 = Kpv1 * Tdv1;
Tfv1 = Tdv1 / 5;

% Define outer controller parameters
Kpp1 = 1500; 
Kip1 = 0;
Kdp1 = 0;

% Define control objects
inner_ctrl1 = PIDController(Tc, Kpv1, Kiv1, Kdv1, [], [], []);
inner_ctrl1.setUMax(840); % limiti torque1 del nostro robot 
outer_ctrl1 = PIDController(Tc, Kpp1, Kip1, Kdp1, [], [], []);
cascade_ctrl1 = CascadeController(Tc, inner_ctrl1, outer_ctrl1);


%  ---------------------------CONTROLLORE 2 --------------------------------------
Kpv2=2019;
Tiv2=0.025;
Kiv2=Kpv2/Tiv2;


Tdv2 = 0.0;
Kdv2 = Kpv2 * Tdv2;
Tfv2 = Tdv2 / 5;



% Define outer controller parameters
Kpp2 = 1000; 
Kip2 = 10;
Kdp2 = 0.0;
inner_ctrl2 = PIDController(Tc, Kpv2, Kiv2, Kdv2, [], [], []);
inner_ctrl2.setUMax(220); %limiti torque2 del nostro robot 
outer_ctrl2 = PIDController(Tc, Kpp2, Kip2, Kdp2, [], [], []);
cascade_ctrl2 = CascadeController(Tc, inner_ctrl2, outer_ctrl2);

USE_DYNPAR=0;
if USE_DYNPAR
    load +scara_data/dynamic_paramers
    idyn=@(q,Dq,DDq)[spong2_ctrl.regressorFcn(q,Dq,DDq) diag(Dq) Dq]*dynamic_parameters;
    decentralized_ctrl = DecentralizedController(Tc,[cascade_ctrl1,cascade_ctrl2],idyn);
else
    decentralized_ctrl = DecentralizedController(Tc,[cascade_ctrl1,cascade_ctrl2]);
end
decentralized_ctrl.initialize;

decentralized_ctrl.starting(zeros(6,1),zeros(4,1),zeros(2,1),zeros(2,1));
u=decentralized_ctrl.computeControlAction(zeros(6,1),zeros(4,1),zeros(2,1));