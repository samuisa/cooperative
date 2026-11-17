classdef TaskAltitudeControl < Task
    %TASKMINALTITUDE Keep the vehicle above a minimum measured altitude.
    %
    % The task controls world-vertical vehicle velocity using the vehicle-frame twist. It remains inactive until a finite altitude measurement is available.

    properties
        feedbackGain = 0.2        % Proportional gain 
        maxLinearVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].
        v_height
        v_height_goal = 0.0;
    end

    methods
        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded Cartesian feedback velocity.
            %
            % Input:
            %   robot - UvmsModel containing the current altitude measurement.
            %
            % Output:
            %   None. Updates obj.xdotbar with the desired world-vertical velocity.
            obj.v_height = robot.altitude;

            if ~isscalar(obj.v_height) || ~isfinite(obj.v_height)
                % checking if altitude value is not Nan (since sensors may
                % not provide data immediatley after the start of the
                % simulation)
                obj.xdotbar = 0;
                return
            end
               
            obj.xdotbar = obj.feedbackGain*(obj.v_height_goal - obj.v_height);
            %obj.xdotbar = Saturate(obj.xdotbar, obj.maxLinearVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the tool Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the vehicle orientation.
            %
            % Output:
            %   None. Updates obj.J, a [1x13] row mapping system velocities to world-vertical vehicle velocity.
            
            if ~isscalar(obj.v_height) || ~isfinite(obj.v_height)
                % checking if altitude value is not Nan(since sensors may
                % not provide data immediatley after the start of the
                % simulation)
                obj.J = zeros(1, numel(robot.q) + 6);
            else
                obj.J = [zeros(1,numel(robot.q)) robot.wTv(3,1) robot.wTv(3,2) robot.wTv(3,3) zeros(1,3)];
            end
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Activate altitude correction near the minimum.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Updates obj.A, a [1x1] altitude activation.
            if ~isscalar(obj.v_height) || ~isfinite(obj.v_height)
                % checking if altitude value is not Nan(since sensors may
                % not provide data immediatley after the start of the
                % simulation)
                obj.A = 0;
            else
                obj.A = 1;
            end
        end
    end
end
