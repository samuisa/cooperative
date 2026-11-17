classdef TaskAttitudeControl < Task
    %TASKATTITUDECONTROL Align the vehicle orientation with its goal.

    properties
        maxAngularVelocity = 0.2  % Maximum absolute component of the angular velocity reference [rad/s].
        feedbackGain = 0.2        % Proportional gain 
        threshold_ = 0.01;
    end

    methods
        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded Cartesian feedback velocity.
            %
            % Input:
            %   robot - UvmsModel with current tool transform robot.vTt and desired tool transform robot.vTg.
            %
            % Output:
            %   None. Updates obj.xdotbar as [angular; linear] velocity, expressed in the vehicle frame.
            goal_orientation = robot.wRgv;
            err = (1/2)* (cross(robot.wTv(1:3,1),goal_orientation(:,1)) + ...
                          cross(robot.wTv(1:3,2),goal_orientation(:,2)) + ...
                          cross(robot.wTv(1:3,3),goal_orientation(:,3)) );
            obj.xdotbar = obj.feedbackGain*(-err);
            %obj.xdotbar = Saturate(obj.xdotbar, obj.maxAngularVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the tool Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.
            
            obj.J = [zeros(3,10) robot.wTv(1:3,1:3)];
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Disable orientation control once the goal is reached.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            goal_orientation = robot.wRgv;
            err = (1/2)* (cross(robot.wTv(1:3,1), goal_orientation(:,1)) + ...
                          cross(robot.wTv(1:3,2), goal_orientation(:,2)) + ...
                          cross(robot.wTv(1:3,3), goal_orientation(:,3)));
            if norm(err) < obj.threshold_
                obj.A = zeros(3);
            else
                obj.A = eye(3);
            end
        end
    end
end
