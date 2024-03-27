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
        function grasped = step(obj, position, velocity, effort)
            grasped=randn>4;
        end
    end
end
