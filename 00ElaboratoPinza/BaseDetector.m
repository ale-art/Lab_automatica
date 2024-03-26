% BaseFilter - Abstract class for implementing digital filters
%
classdef (Abstract)  BaseDetector < handle & matlab.mixin.Heterogeneous
    properties (Access = protected)
        Tc % Sampling time (cycle time)
    end
    
    methods (Access = public)
        % Constructor for BaseFilter class
        function obj = BaseDetector(Tc)
            % Validate that Tc is a scalar and greater than zero
            assert(isscalar(Tc), 'Sampling time (Tc) must be a scalar');
            assert(Tc > 0, 'Sampling time (Tc) must be greater than zero');
            
            % Assign the sampling time to the property
            obj.Tc = Tc;
        end
    end

    methods (Abstract, Access = public)
        % Abstract methods to be implemented in derived classes

        % Initialize variables
        obj = initialize(obj)
        
        % Initialize the object to start 
        obj = starting(obj, position, velocity, effort)

        % Compute if the gripper is grasping
        grasped = step(obj, position, velocity, effort)
    end
end
