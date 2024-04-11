clear all;close all;clc

load tutte_le_prove.mat

for it=1:length(tests)

    vel=tests(it).velocity;
    pos=tests(it).position;
    eff=tests(it).effort;

  
    figure
    subplot(311)
    plot(tests(it).time,pos)
    grid on
    xlabel('t')
    ylabel('p [m]')

    subplot(312)
    plot(tests(it).time,vel)
    grid on
    xlabel('t')
    ylabel('v [m/s]')


    subplot(313)
    plot(tests(it).time,eff)
    grid on
    xlabel('t')
    ylabel('effort [N]')

end