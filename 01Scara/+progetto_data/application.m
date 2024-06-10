
clear all; close all; clc;


% Set the maximum velocities (max_Dy) and maximum accelerations (max_DDy)
% tune them to improve the score

folder='progetto1';
robot=ElasticRoboticSystem(folder);
robot.initialize;

% Set the cycle time (sampling time) for motion law updates
Tc = robot.getSamplingPeriod;

%load("+progetto_data"+filesep+"dummy_controller_object")
load decentralized_ctrl_j1j2_v1.mat

show_figure=false;

% da modificare per alzare score --> % totale score 85.7%
max_Dcart =  [15,15]'; 
max_DDcart = [10,10]';

ctrl=decentralized_ctrl;
ctrl.setUMax(robot.getUMax)

ntrials=3; % for each program run ntrials

programs=["+progetto_data/square.code","+progetto_data/diamond.code","+progetto_data/decagon.code"];
for  iprogram=1:length(programs)
    program_name=programs(iprogram);


    for itrial=1:3
        ctrl.initialize;
        robot.initialize;
        measured_output = robot.readSensorValue();
        q0=measured_output(1:2);
        Dq0=measured_output(3:4);
        DDq0=[0;0];

        % initial reference is equal to the inital state of the robot
        initial_reference=[q0;Dq0;DDq0];

        T_base_tool=spong2_ctrl.fkFcn(q0);

        initial_cartesian_reference=T_base_tool([1 3],4);

        % define the Motion law
        ml = TrapezoidalMotionLaw(max_Dcart, max_DDcart, Tc);
        ml.setInitialCondition(initial_cartesian_reference);

        % Define a sequence of motion instructions
        instructions=readlines(program_name);
        ml.addInstructions(instructions);

        % define direct and inverse kinematics solver
        ikm=IkMotion(Tc,q0,@spong2_ctrl.jacobFcn,@spong2_ctrl.jacobDotFcn,@spong2_ctrl.ikFcn,@spong2_ctrl.fkFcn);

        % read the initial torque
        joint_torque=robot.readActuatorValue;

        % feedforward action, not used in this script
        feedforward_action=[0;0];

        % starting controller
        ctrl.starting(initial_reference,measured_output,joint_torque,feedforward_action);

        % clear variable in each run
        clear target_cartesian target_Dcartesian target_DDcartesian t
        clear measured_signal control_action reference_signal full_position

        % execute the code
        for idx = 1:600e3

            % compute Cartesian target motion
            [target_cartesian(idx, :), target_Dcartesian(idx, :), target_DDcartesian(idx, :)] = ml.computeMotionLaw();
            % compute joint target motion
            [target_q(idx,:),target_Dq(idx,:),target_DDq(idx,:)]=ikm.ik(target_cartesian(idx,:)',target_Dcartesian(idx,:)',target_DDcartesian(idx,:)');
            % define reference
            reference=[target_q(idx,:),target_Dq(idx,:),target_DDq(idx,:)]';

            % y joint position from encoder
            measured_output = robot.readSensorValue();

            % compute control law
            joint_torque = ctrl.computeControlAction(reference,measured_output,feedforward_action);

            % apply torque
            robot.writeActuatorValue(joint_torque);

            % simulate
            robot.simulate();

            % store variable for evaluation and plotting
            t(idx,:)=(idx-1)*Tc;
            measured_signal(idx,:)=measured_output;
            control_action(idx,:)=joint_torque;
            reference_signal(idx,:)=reference;
            full_position(idx,:)=robot.fullJointPosition;
            % Check if there are more instructions to follow
            if ~ml.dependingInstructions
                break;
            end
        end


        %%
        joint_position=measured_signal(:,[1 2]);
        joint_velocity=measured_signal(:,[3 4]);

        % acceleration is computed Savitzky-Golay filter
        joint_acceleration = zeros(length(joint_velocity),size(joint_position,2));
        window=2;
        [b,g] = sgolay(1,1+2*window);

        g_filter=g(:,1)'; % moving average
        g_filter_der=g(:,2)'/Tc; % first derivative

        for iax = 1:size(joint_position,2)
            for idx = (1+window):(size(joint_position,1)-window)
                joint_acceleration(idx,iax) = g_filter_der*joint_velocity((idx-window):(idx+window),iax);
            end
        end

        reference_position=reference_signal(:,[1 2]);
        reference_velocity=reference_signal(:,[3 4]);
        reference_acceleration=reference_signal(:,[5 6]);

        if show_figure
            figure
            subplot(4,2,1)
            plot(t,joint_position(:,1),t,reference_position(:,1))
            hold on
            grid on
            xlabel('Time')
            ylabel('Position 1')
            legend({'Position','Position reference'})

            subplot(4,2,3)
            plot(t,joint_velocity(:,1),t,reference_velocity(:,1))
            hold on
            grid on
            xlabel('Time')
            ylabel('Velocity 1')
            legend({'Velocity','Velocity reference'})

            subplot(4,2,5)
            plot(t,joint_acceleration(:,1),t,reference_acceleration(:,1))
            hold on
            grid on
            xlabel('Time')
            ylabel('Acceleration 1')
            legend({'Acceleration','Acceleration reference'})

            subplot(4,2,7)
            plot(t,control_action(:,1))
            hold on
            grid on
            xlabel('Time')
            ylabel('Torque 1')
            legend({'Torque'})


            subplot(4,2,2)
            plot(t,joint_position(:,2),t,reference_position(:,2))
            hold on
            grid on
            xlabel('Time')
            ylabel('Position 2')
            legend({'Position','Position reference'})

            subplot(4,2,4)
            plot(t,joint_velocity(:,2),t,reference_velocity(:,2))
            hold on
            grid on
            xlabel('Time')
            ylabel('Velocity 2')
            legend({'Velocity','Velocity reference'})

            subplot(4,2,6)
            plot(t,joint_acceleration(:,2),t,reference_acceleration(:,2))
            hold on
            grid on
            xlabel('Time')
            ylabel('Acceleration 2')
            legend({'Acceleration','Acceleration reference'})

            subplot(4,2,8)
            plot(t,control_action(:,2))
            hold on
            grid on
            xlabel('Time')
            ylabel('Torque 2')
            legend({'Torque'})


            title(program_name)
            drawnow
        end
        itest=(iprogram-1)*ntrials+itrial;
        test_data(itest).full_position=full_position;
        test_data(itest).measured_signal=measured_signal;
        test_data(itest).control_action=control_action;
        test_data(itest).time=t;
        test_data(itest).target_cartesian=target_cartesian;
        test_data(itest).name=program_name;

    end
end

load("+"+folder+filesep+"evaluation_data")
evaluator=progetto_data.Evaluator(Tc,@spong2.fkSimulation,@spong2_ctrl.fkFcn,expected_total_time,cartesian_distance_penalty,show_figure);
score=evaluator.evaluate(test_data);
fprintf('Score = %f\n',score);

