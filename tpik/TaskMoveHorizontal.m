classdef TaskMoveHorizontal < Task
    %TASKTOOL Six-dimensional Cartesian pose task for the UVMS tool.
    %
    % The task drives the tool frame vTt toward the goal frame vTg. Both frames are expressed relative to the vehicle, so the resulting error, reference velocity and Jacobian use a consistent vehicle-frame representation. All six components are permanently active.

    properties
        feedbackGain = 0.2        % Proportional gain that converts Cartesian error into a velocity reference [1/s].
        maxAngularVelocity = 0.2  % Maximum absolute component of the angular velocity reference [rad/s].
        maxLinearVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].
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

            % x(c) is the misalignment of z vehicle vector wrt z-axis world frame,
            % expressed as projection of z vector on x and y axes of world
            % frame. The goal is considered to have misalignment zero,
            % however, when updating the activation function, the 0.2
            % constraint will be considered.
            obj.xdotbar = -obj.feedbackGain*[robot.wTv(1,3); robot.wTv(2,3)];

            % Limit angular and linear parts independently while preserving the direction within each three-dimensional subvector.
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

            % Vehicle motion affects the tool both directly and through the lever arm from the vehicle origin to the tool position.
            J_1 = [zeros(1,10) -robot.wTv(1,2) robot.wTv(1,1) 0];
            J_2 = [zeros(1,10) -robot.wTv(2,2) robot.wTv(2,1) 0];

            % Concatenate according to ydot = [q_dot; v_nu].
            obj.J = [J_1; J_2];
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all Cartesian components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            %#ok<INUSD>
            x = [robot.wTv(1,3) robot.wTv(2,3)];
            x_norm = norm(x);
            delta = 0.05;
            s = (x_norm - (0.2-delta))/delta;

            if x_norm > 0.2
                obj.A = eye(2);
            elseif x_norm <= 0.2 && x_norm >= (0.2 - delta)
                obj.A = eye(2)*s;
            else
                obj.A = zeros(2);
            end
        end
    end
end

