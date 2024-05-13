clear all;close all;clc

load tutte_le_prove.mat
% load prove_con_presa_oggetto.mat
% load prove_senza_presa_oggetto.mat

% Load the parameters found using the calcoloParametri.m script
load parametri_tot.mat

% Remove the invalid tests
tests(2) = []; tests(4) = [];

Tc = 2e-3; % Sampling time [s]

% Initialize our detector
detector=GraspDetector(Tc); 
window = 19; % half window, also try = 15

% Thresholds for speed, effort and accelaration 
thr_vel = 3e-3;
thr_eff = 0.15;
thr_acc = -0.0035;

for it=1:length(tests)

    grasp=zeros(length(tests(it).time),1);
    p_window = []; v_window = []; e_window = [];
    p_filt = []; v_filt = []; a_filt = []; e_filt =[];
    e_mod = [];

    for idx=1:length(tests(it).time)
        p=tests(it).position(idx);
        v=tests(it).velocity(idx);
        e=tests(it).effort(idx);
        if idx==1
            detector.starting(p,v,e);
        end

        %%%%%%%%%%%%%%%%%%
        if idx<=(1+2*window)
            p_window = [p_window; p];
            v_window = [v_window; v];
            e_window = [e_window; e];
        else
            % delayed of window index
            [p_filt_temp, v_filt_temp, a_filt_temp, e_filt_temp] = detector.filter(window, p_window, v_window, e_window);
            p_filt = [p_filt; p_filt_temp];
            v_filt = [v_filt; v_filt_temp];
            a_filt = [a_filt; a_filt_temp];
            e_filt = [e_filt; e_filt_temp];

            % shift the window
            p_window = [p_window; p];
            p_window = p_window(2:end);
            v_window = [v_window; v];
            v_window = v_window(2:end);
            e_window = [e_window; e];
            e_window = e_window(2:end);

            % model effort
            e_mod_temp = [a_filt_temp v_filt_temp tanh(1000*v_filt_temp)] * parametri_tot;
            e_mod = [e_mod; e_mod_temp];
            
            grasp(idx,1) = detector.step(v_filt_temp,a_filt_temp,e_filt_temp,e_mod_temp, ...
                thr_vel,thr_eff,thr_acc); 
        end
    end

    index_min = find(grasp==1,1,'first');
    grasp_plot = zeros(length(tests(it).time),1);
    grasp_plot(index_min) = 1;
    % 
    % figure
    % subplot(411)
    % plot(tests(it).time,tests(it).position)
    % grid on
    % xlabel('t')
    % ylabel('p [m]')
    % 
    % subplot(412)
    % plot(tests(it).time,tests(it).velocity)
    % grid on
    % xlabel('t')
    % ylabel('v [m/s]')
    % 
    % 
    % subplot(413)
    % plot(tests(it).time,tests(it).effort)
    % grid on
    % xlabel('t')
    % ylabel('effort [N]')
    % 
    % subplot(414)
    % plot(tests(it).time,grasp)
    % grid on
    % xlabel('t')
    % ylabel('Grasped?')
    
    zeros_supp = zeros(window,1);
    %time_filt = tests(it).time(1:idx);
    
    figure();
    subplot(411)
    plot(tests(it).time,tests(it).velocity)
    grid on
    xlabel('t')
    ylabel('v [m/s]')
    hold on
    v_filt = [zeros_supp; v_filt; zeros_supp; 0];
    plot(tests(it).time,v_filt)
    hold on
    yline(thr_vel)
    
    subplot(412)
    a_filt = [zeros_supp; a_filt; zeros_supp; 0];
    plot(tests(it).time,a_filt)
    grid on
    xlabel('t')
    ylabel('a [m/s^2]')
    hold on
    yline(thr_acc)
    
    subplot(413)
    % plot(tests(it).time,tests(it).effort)
    grid on
    xlabel('t')
    ylabel('effort [N]')
    hold on
    e_filt = [zeros_supp; e_filt; zeros_supp; 0];
    plot(tests(it).time,e_filt)
    hold on 
    e_mod = [zeros_supp; e_mod; zeros_supp; 0];
    plot(tests(it).time,e_mod)

    subplot(414)
    plot(tests(it).time,grasp_plot)
    grid on
    xlabel('t')
    ylabel('Grasped?')

end
