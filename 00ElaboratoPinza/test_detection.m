clear all;close all;clc

% load tutte_le_prove.mat
load prove_con_presa_oggetto.mat
% load prove_senza_presa_oggetto.mat
load parametri_tot.mat

Tc = 2e-3;
% Change Dummy with your detector
detector=DummyDetector(Tc);
window = 15; % half window, also try = 7

p_window = []; v_window = []; e_window = [];
p_filt = []; v_filt = []; a_filt = []; e_filt =[];
e_mod = [];


for it=5:5 % da mettere length(tests)

    grasp=zeros(length(tests(it).time),1);
    for idx=1:length(tests(it).time)
        p=tests(it).position(idx);
        v=tests(it).velocity(idx);
        e=tests(it).effort(idx);
        if idx==1
            detector.starting(p,v,e);
        end

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
            grasp(idx,1)=detector.step(p,v,e,e_mod);
        end

        if (grasp(idx,1))
            break
        end
    end

    figure
    subplot(411)
    plot(tests(it).time,tests(it).position)
    grid on
    xlabel('t')
    ylabel('p [m]')

    subplot(412)
    plot(tests(it).time,tests(it).velocity)
    grid on
    xlabel('t')
    ylabel('v [m/s]')


    subplot(413)
    plot(tests(it).time,tests(it).effort)
    grid on
    xlabel('t')
    ylabel('effort [N]')

    subplot(414)
    plot(tests(it).time,grasp)
    grid on
    xlabel('t')
    ylabel('Grasped?')
end

%% plot test filter

zeros_supp = zeros(window,1);

figure();
subplot(311)
plot(tests(it).time,tests(it).velocity)
grid on
xlabel('t')
ylabel('v [m/s]')
hold on
v_filt = [zeros_supp; v_filt; zeros_supp; 0];
plot(tests(it).time,v_filt)

subplot(312)
a_filt = [zeros_supp; a_filt; zeros_supp; 0];
plot(tests(it).time,a_filt)
grid on
xlabel('t')
ylabel('a [m/s^2]')


subplot(313)
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

