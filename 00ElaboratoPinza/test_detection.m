clear all;close all;clc

load tutte_le_prove.mat


% Change Dummy with your detector
detector=DummyDetector(2e-3);

for it=1:length(tests)

    grasp=zeros(length(tests(it).time),1);
    for idx=1:length(tests(it).time)
        p=tests(it).position(idx);
        v=tests(it).velocity(idx);
        e=tests(it).effort(idx);

        if idx==1
            detector.starting(p,v,e);
        end
        
        grasp(idx,1)=detector.step(p,v,e);
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