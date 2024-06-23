classdef Evaluator < handle
    properties  (Access = protected)
        Tc; % Cycle time
        fkElastic;
        fkRigid;

        cartesian_distance_penalty;
        time_penalty=50;
        expected_total_time;
        show_figure
    end

    methods  (Access = public)
        function obj = Evaluator(Tc,fkElastic,fkRigid,expected_total_time,cartesian_distance_penalty,show_figure)
            obj.Tc=Tc;
            obj.fkElastic=fkElastic;
            obj.fkRigid=fkRigid;

            keySet = {'decagon.code' 'diamond.code' 'square.code'};
            obj.expected_total_time = containers.Map(keySet,expected_total_time);
            obj.cartesian_distance_penalty = containers.Map(keySet,cartesian_distance_penalty);
            obj.show_figure=show_figure;
        end


        function score=evaluate(obj,tests_data)
            for itest=1:length(tests_data)
                scores(itest,1)=obj.evaluateSingleTest(tests_data(itest));
            end
            scores=sort(scores);
            score=mean(scores(2:end-1));
        end
    end

    methods  (Access = protected)
        function score=evaluateSingleTest(obj,test_data)
            cartesian_poses=zeros(size(test_data.full_position,1),2);
            cartesian_poses_from_encoder=zeros(size(test_data.full_position,1),2);
            max_cartesian_distance=-1;


            % overfit target_cartesian to estimate better the distance
            t=linspace(test_data.time(1),test_data.time(end),length(test_data.time)*50)';
            target_cartesian=interp1(test_data.time,test_data.target_cartesian,t);


            idx1=find(test_data.time>0.4,1,'first');
            for idx=1:size(test_data.full_position,1)
                T=obj.fkElastic(test_data.full_position(idx,:)');
                cartesian_poses(idx,:)=T([1 3],4);

                T=obj.fkRigid(test_data.measured_signal(idx,1:2)');
                cartesian_poses_from_encoder(idx,:)=T([1 3],4);

                if idx<idx1
                    continue
                end
                distance=sqrt(...
                    (target_cartesian(:,1)-cartesian_poses(idx,1)).^2+...
                    (target_cartesian(:,2)-cartesian_poses(idx,2)).^2);
                if min(distance)>max_cartesian_distance
                    [max_cartesian_distance,i_max_dist2]=min(distance);
                    i_max_dist1=idx;
                end
            end


            

            total_time=test_data.time(end);

            test_name= extractAfter(test_data.name,'/');
            diff_time=max(0,total_time-obj.expected_total_time(test_name))/obj.expected_total_time(test_name);

            max_cartesian_distance=max(0,max_cartesian_distance-5e-4); % un errore sotto gli 7e-4 è considerato trascurabile
            score = 100 -  ...
                diff_time*obj.time_penalty - ...
                max_cartesian_distance*obj.cartesian_distance_penalty(test_name);

            if obj.show_figure
                figure
                plot(target_cartesian(:,1),target_cartesian(:,2),'--k')
                hold on
                plot(cartesian_poses_from_encoder(:,1),cartesian_poses_from_encoder(:,2),'--r')
                plot(cartesian_poses(:,1),cartesian_poses(:,2),'b')
                plot([cartesian_poses(i_max_dist1,1) target_cartesian(i_max_dist2,1)],...
                    [cartesian_poses(i_max_dist1,2) target_cartesian(i_max_dist2,2)],'-d','LineWidth',2)
                axis equal;
                grid on
                legend('Target Cartesian position','Cartesian position from encoder','Actual Cartesian position','maximum distance')
                title(sprintf('Score = %f',score))
            end
            fprintf('Score = %f, extra time (%%) = %f, max Cartesian distance = %e\n',score,diff_time*100,max_cartesian_distance);
        end
    end
end