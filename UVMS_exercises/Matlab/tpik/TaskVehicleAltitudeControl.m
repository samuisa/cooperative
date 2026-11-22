classdef TaskVehicleAltitudeControl < Task
    properties
        feedGain = 0.5;
        targetAltitude = 0.0;
    end
    
    methods
        function updateReference(obj, robot)
            if isnan(robot.altitude)
                obj.xdotbar = 0;
            else
                err_w = obj.targetAltitude - robot.altitude; 
                obj.xdotbar = obj.feedGain * err_w;
            end
        end
        
        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3, 1:3); 
            obj.J = [zeros(1, 7), [0, 0, 1] * wRv, zeros(1, 3)];
        end
        
        function updateActivation(obj, robot)
            if isnan(robot.altitude)
                obj.A = 0;
            else
                obj.A = 1;
            end
        end
    end
end