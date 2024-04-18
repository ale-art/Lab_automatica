clear all;close all;clc

load tutte_le_prove.mat

Tc = 2e-3;
% Change Dummy with your detector
detector=DummyDetector(Tc);
window = 7;
[b,g] = sgolay(1,1+2*window);
g_filter=g(:,1)'; % moving average
g_filter_der=g(:,2)'/Tc; % first derivative

for it=1:1

    grasp=zeros(length(tests(it).time),1);
    for idx=1:length(tests(it).time)
        p=tests(it).position(idx);
        v=tests(it).velocity(idx);
        e=tests(it).effort(idx);
        p_vector(idx) = p; v_vector(idx) = v; e_effort(idx)=e;
        if idx==1
            detector.starting(p,v,e);
        end

        if (mod(idx,15)==0)
%           for iax = 1:15
%             for id = (1+window):(15-window)
%                 gripper_position_filt(id,iax) = g_filter*gripper_position((id-window):(id+window),iax);
%                 gripper_velocity_filt(id,iax) = g_filter*gripper_velocity((id-window):(id+window),iax);
%                 gripper_effort_filt(id,iax) = g_filter*gripper_effort((id-window):(id+window),iax);
%                 gripper_acceleration_filt(id,iax) = g_filter_der*gripper_velocity((id-window):(id+window),iax);
%             end
%           end
        
          clear p_vector
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