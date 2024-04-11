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
        function grasped = step(obj, position, velocity, acceleration, effort)
            % algoritmo detection presa
            th_eff = 0.38;
            th_acc = -0.025;
            if (effort > th_eff && acceleration < th_acc)
                grasped=1;
            else
                grasped=0;
            end
        end
    end
end
