classdef TaskStop < Task
    %TASKSTOP Fourteen-dimensional joint-space task setting all arm joint velocities to zero.

    methods
        function obj = TaskStop(taskName)
            obj.taskName = string(taskName);
        end

        function updateReference(obj)
            obj.xdotbar = zeros(14, 1);
        end

        function updateJacobian(obj)
            obj.J = eye(14);
        end

        function updateActivation(obj)
            obj.A = eye(14);
        end
    end
end