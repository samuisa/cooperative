classdef TaskHorizontalPos < Task
    %TASKTOOL Six-dimensional Cartesian pose task for the UVMS tool.
    %
    % The task drives the tool frame vTt toward the goal frame vTg. Both frames are expressed relative to the vehicle, so the resulting error, reference velocity and Jacobian use a consistent vehicle-frame representation. All six components are permanently active.

    properties
        feedbackGain = 0.2        % Proportional gain 
        maxLinearVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].
        maxAngularVelocity = 0.2   % Maximum absolute component of the linear velocity reference [m/s].
        theta
        w_rho %since i will need unit vector of rho, w since projected on w
        theta_max = 0.2;
        delta = 0.1; %from 0.1 to 0.2 in activation function i have the transition
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

            % compute cross prod(versor lemma -> theta and out[output betwen two rot matrices], but can 
            % use reduced versor lemma which gives theta and rot vector normalized])
            % then take norm, so i can do it on any frame
            obj.w_rho = ReducedVersorLemma([0 0 1]',robot.wTv(1:3,3)); % passing kw[0 0 1] and kv, ->
            % computes the rotation vector (there is additional sin(theta), otherwise i dont get theta)
            obj.theta = norm(obj.w_rho);
            obj.xdotbar = Saturate(obj.feedbackGain*((obj.theta_max-obj.delta)-obj.theta), obj.maxAngularVelocity); %theta star - theta
        end

        function updateJacobian(obj, robot)
            %UPDATEJACOBIAN Build the tool Jacobian for arm and vehicle motion.
            %
            % Input:
            %   robot - UvmsModel providing q and the transforms vTb, vTe, vTt and eTt.
            %
            % Output:
            %   None. Updates obj.J, a [6x13] matrix with columns ordered as [seven arm joints, six vehicle velocities]. Rows follow the [angular; linear] convention used by obj.xdotbar.
            
            if (obj.theta ~= 0)
                w_n = obj.w_rho/obj.theta; % write where vector are projected -> w_n and w_rho projected on world frame
                % jacobian not projected anywhere since its scalar, but one
                % when w_n' multiplied with wRv need to be careful to
                % frames
                obj.J = w_n'*[zeros(3,numel(robot.q)) zeros(3,3) robot.wTv(1:3,1:3)];
            else
                obj.J = zeros(1, numel(robot.q) + 6);
            end
        end

        function updateActivation(obj, robot)
            %UPDATEACTIVATION Keep all Cartesian components fully active.
            %
            % Input:
            %   robot - UvmsModel (unused; retained for the common Task API).
            %
            % Output:
            %   None. Sets obj.A to the [6x6] identity matrix.
            obj.A = IncreasingBellShapedFunction(obj.theta_max - obj.delta, obj.theta_max, 0, 1, obj.theta);
        end
    end
end

