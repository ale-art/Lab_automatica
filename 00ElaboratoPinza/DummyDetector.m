classdef DummyDetector < BaseDetector

    properties (Access = protected)
    end
    
    methods (Access = public)
        function obj = DummyDetector(Tc)
            obj@BaseDetector(Tc);
        end


        function obj = initialize(obj)
        end

        function obj = starting(obj, position, velocity, effort)
        end

        % Compute if the gripper is grasping
        % function grasped = step(obj, position, velocity, acceleration, effort)
        %     % algoritmo detection presa
        %     th_eff = 0.38;
        %     th_acc = -0.025;
        %     if (effort > th_eff && acceleration < th_acc)
        %         grasped=1;
        %     else
        %         grasped=0;
        %     end
        % end
        
        function [p_filt, v_filt, a_filt, e_filt] = filter(obj, window, position, velocity, effort)
            [b,g] = sgolay(1,1+2*window);
            g_filter=g(:,1)'; % moving average
            g_filter_der=g(:,2)'/obj.Tc; % first derivative
            % iax not used
            for index = (1+window):(size(position,1)-window)
                p_filt = g_filter*position((index-window):(index+window));
                v_filt = g_filter*velocity((index-window):(index+window));
                e_filt = g_filter*effort((index-window):(index+window));
                a_filt = g_filter_der*velocity((index-window):(index+window));
            end
        end

        function grasped = step(obj, velocity, acceleration, effort, effort_model)
            soglia_vel = 1e-5;
            soglia_acc = -0.003;
            soglia_effort = 0.15;
            if abs(velocity) < soglia_vel
                  grasped = 0;
            elseif acceleration > soglia_acc
                  grasped = 0;
            elseif abs(abs(effort_model)-abs(effort)) > soglia_effort
                  grasped = 1;
            else
                  grasped = 0;
            end

        end

    end
end
