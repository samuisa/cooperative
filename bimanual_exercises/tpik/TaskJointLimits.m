classdef TaskJointLimits < Task
    %TASKJOINTLIMITS Seven-dimensional inequality task keeping arm joints within [jlmin, jlmax].

    properties
        robot                   % PandaArm whose joint limits are protected.
        armID = ""              % "left" or "right"; selects the columns in the 14-DOF control vector.
        activationBuffer = 0.05 % Safety interval [rad] inside jlmin/jlmax where smooth activation occurs.
        feedbackGain = 1        % Proportional gain [1/s].
    end

    methods
        function obj = TaskJointLimits(robot, armID)
            obj.robot = robot;
            obj.armID = string(armID);
            obj.taskName = obj.armID + " joint limits";
        end

        function updateReference(obj)
            q = obj.robot.q;
            lowerThreshold = obj.robot.jlmin + obj.activationBuffer;
            upperThreshold = obj.robot.jlmax - obj.activationBuffer;

            % Positive rate when below lowerThreshold; negative rate when above upperThreshold.
            lowerError = max(lowerThreshold - q, 0);
            upperError = min(upperThreshold - q, 0);
            obj.xdotbar = obj.feedbackGain * (lowerError + upperError);
        end

        function updateJacobian(obj)
            if obj.armID == "left"
                obj.J = [eye(7), zeros(7, 7)];
            else
                obj.J = [zeros(7, 7), eye(7)];
            end
        end

        function updateActivation(obj)
            q = obj.robot.q;
            jlmin = obj.robot.jlmin;
            jlmax = obj.robot.jlmax;
            a = zeros(7, 1);

            for i = 1:7
                aLow  = DecreasingBellShapedFunction(jlmin(i), jlmin(i) + obj.activationBuffer, 0, 1, q(i));
                aHigh = IncreasingBellShapedFunction(jlmax(i) - obj.activationBuffer, jlmax(i), 0, 1, q(i));
                a(i) = max(aLow, aHigh);
            end
            obj.A = diag(a);
        end
    end
end