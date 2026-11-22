classdef TaskMinAltitude < Task
    %TASKMINALTITUDE Inequality task keeping one Panda tool above a minimum table altitude.

    properties
        robot                  % PandaArm whose tool altitude defines this task.
        armID = ""             % "left" or "right"; selects the Jacobian columns in the 14-DOF vector.
        feedGain = 0.5         % Proportional gain [1/s].
        altMin = 0.15          % Minimum allowed altitude [m].
        altThreshold = 0.18    % Altitude below which smooth activation begins [m].
    end

    methods
        function obj = TaskMinAltitude(robot, armID)
            obj.robot = robot;
            obj.armID = string(armID);
        end

        function updateReference(obj)
            currentAlt = obj.robot.wTt(3, 4);
            obj.xdotbar = obj.feedGain * (obj.altMin - currentAlt);
        end

        function updateJacobian(obj)
            % Row 6 of wJt corresponds to linear z velocity in the world frame.
            if obj.armID == "left"
                obj.J = [obj.robot.wJt(6, :) zeros(1, 7)];
            else
                obj.J = [zeros(1, 7) obj.robot.wJt(6, :)];
            end
        end

        function updateActivation(obj)
            currentAlt = obj.robot.wTt(3, 4);
            % Smooth activation in [altMin, altThreshold]: 1 below altMin, 0 above altThreshold.
            obj.A = DecreasingBellShapedFunction(obj.altMin, obj.altThreshold, 0, 1, currentAlt);
        end
    end
end