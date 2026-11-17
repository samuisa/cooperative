classdef TaskVehiclePosition < Task
    %TASKVEHICLEPOSITION Three-dimensional vehicle position task.
    %
    % The task drives the vehicle position in the world frame toward its goal.
    % Vehicle orientation is not part of this task.

    properties
        feedbackGain = 0.2        % Proportional gain that converts Cartesian error into a velocity reference [1/s]. cannot be too high to avoid oscillation and saturation (ex 3)
        maxLinearVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].
    end

    methods
        function updateReference(obj, robot)
            %UPDATEREFERENCE Build a bounded Cartesian feedback velocity.
            %
            % Input:
            %   robot - UvmsModel with current vehicle pose robot.wTv and desired pose robot.wTgv.
            %
            % Output:
            %   None. Updates obj.xdotbar as the desired world-frame vehicle
            %   position velocity.

            obj.xdotbar = obj.feedbackGain*(robot.wTgv(1:3,4) - robot.wTv(1:3,4));

            % Limit linear part while preserving the direction within the three-dimensional subvector.
            %obj.xdotbar = Saturate(obj.xdotbar, obj.maxLinearVelocity);
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Map vehicle body-frame translation to world position velocity.
            %
            % Input:
            %   robot - UvmsModel with the current vehicle pose robot.wTv.
            %
            % Output:
            %   None. Updates obj.J, a [3x13] matrix with columns ordered
            %   as [seven arm joints, three vehicle linear velocities, three
            %   vehicle angular velocities].
            obj.J = [zeros(3,7) robot.wTv(1:3,1:3) zeros(3,3)];

        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all three position components active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [3x3] identity matrix.
            %#ok<INUSD>
            obj.A = eye(3);
        end
    end
end

