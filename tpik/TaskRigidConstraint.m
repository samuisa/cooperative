classdef TaskRigidConstraint < Task
    %TASKRIGIDCONSTRAINT Six-dimensional binary rigid-grasp constraint between both arms.

    properties
        leftArm                  % Left PandaArm handle.
        rightArm                 % Right PandaArm handle.
        tLo = eye(4)             % [4x4] constant object pose in the left tool frame.
        tRo = eye(4)             % [4x4] constant object pose in the right tool frame.
        isActive = true          % Binary activation flag (no smooth transition).
    end

    methods
        function obj = TaskRigidConstraint(leftArm, rightArm, taskName)
            obj.leftArm = leftArm;
            obj.rightArm = rightArm;
            obj.taskName = string(taskName);
        end

        function computeGraspTransforms(obj)
            %COMPUTEGRASPTRANSFORMS Compute constant tool-to-object transforms right after grasping.
            obj.tLo = obj.leftArm.wTt \ obj.leftArm.wTo;
            obj.tRo = obj.rightArm.wTt \ obj.rightArm.wTo;
        end

        function updateReference(obj)
            % Enforce zero relative Cartesian velocity between the two object frames.
            obj.xdotbar = zeros(6, 1);
        end

        function updateJacobian(obj)
            % Shift left tool Jacobian to the object origin.
            wRtL = obj.leftArm.wTt(1:3, 1:3);
            tLpo = obj.tLo(1:3, 4);
            StoL = [eye(3), zeros(3); -skew(wRtL * tLpo), eye(3)];
            wJoL = StoL * obj.leftArm.wJt;

            % Shift right tool Jacobian to the object origin.
            wRtR = obj.rightArm.wTt(1:3, 1:3);
            tRpo = obj.tRo(1:3, 4);
            StoR = [eye(3), zeros(3); -skew(wRtR * tRpo), eye(3)];
            wJoR = StoR * obj.rightArm.wJt;

            obj.J = [wJoL, -wJoR];
        end

        function updateActivation(obj)
            % Binary activation: either the rigid constraint exists or not.
            if obj.isActive
                obj.A = eye(6);
            else
                obj.A = zeros(6);
            end
        end
    end
end