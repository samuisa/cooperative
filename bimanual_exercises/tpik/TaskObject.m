classdef TaskObject < Task
    %TASKOBJECT Six-dimensional Cartesian pose task for moving the grasped object to wTog.

    properties
        robot                     % Reference PandaArm handle (left arm).
        tTo = eye(4)              % [4x4] constant object pose in the tool frame.
        feedbackGain = 1          % Proportional gain [1/s].
        maxAngularVelocity = 0.3  % Maximum angular velocity [rad/s].
        maxLinearVelocity = 0.3   % Maximum linear velocity [m/s].
    end

    methods
        function obj = TaskObject(robot, taskName)
            obj.robot = robot;
            obj.taskName = string(taskName);
        end

        function computeGraspTransform(obj)
            %COMPUTEGRASPTRANSFORM Store constant tool-to-object transform right after grasping.
            obj.tTo = obj.robot.wTt \ obj.robot.wTo;
        end

        function updateReference(obj)
            wToCurrent = obj.robot.wTt * obj.tTo;
            [angularError, linearError] = CartesianError(obj.robot.wTog, wToCurrent);
            obj.xdotbar = obj.feedbackGain * [angularError; linearError];
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), obj.maxAngularVelocity);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), obj.maxLinearVelocity);
        end

        function updateJacobian(obj)
            wRt = obj.robot.wTt(1:3, 1:3);
            tpo = obj.tTo(1:3, 4);
            Sto = [eye(3), zeros(3); -skew(wRt * tpo), eye(3)];
            wJo = Sto * obj.robot.wJt;

            obj.J = [wJo, zeros(6, 7)];
        end

        function updateActivation(obj)
            obj.A = eye(6);
        end
    end
end