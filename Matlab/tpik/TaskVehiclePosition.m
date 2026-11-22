classdef TaskVehiclePosition < Task
    properties
        feedGain = 0.5;
        tolerance = 1e-3;
    end
    methods
        function updateReference(obj, robot)
            err_w = robot.vehicleGoalPosition - robot.eta(1:3);
            
            obj.xdotbar = obj.feedGain * err_w;
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3);
            
            obj.J = [zeros(3, 7), wRv, zeros(3, 3)];
        end
        
        function updateActivation(obj, robot)
            err_norm = norm(robot.vehicleGoalPosition - robot.eta(1:3));
            
            a = IncreasingBellShapedFunction(1e-3, 0.05, 0, 1, err_norm);
            
            obj.A = a * eye(3);
        end
    end
end