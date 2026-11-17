    %% ROBUST UVMS TASK-PRIORITY EXERCISE
% This script assembles the main blocks of the control architecture:
%
%   task definitions -> ActionManager/iCAT -> kinematic simulator -> Unity
%
% At every iteration, the Kinematic Control Layer computes the system velocity ydot = [q_dot; v_nu].
% UvmsSim integrates this reference directly; in a real robot, a Dynamic Control Layer would track the same reference.
%
% Run this script with the current MATLAB folder set to the template root so that the relative paths below resolve consistently.

%% Make exercise dependencies visible to MATLAB
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tpik');
clc; clear; close all;

%% Configure and initialize the simulation
% dt is both the integration step and the nominal controller sample time.
dt       = 0.005;
endTime  = 50;

% UvmsModel owns state and kinematics; UvmsSim updates that shared handle.
robotModel = UvmsModel();
sim = UvmsSim(dt, robotModel, endTime);

% Unity is used only as viewer/sensor endpoint; it is not the simulator.
unity = UnityInterface("127.0.0.1");

%% Define one action as a strict-priority stack of tasks
% The first cell is the highest-priority task. Add safety tasks before the action objective when extending this exercise (for example, joint limits).
task_tool = TaskTool(); % default task present in script
task_vehicle_pos = TaskVehiclePosition(); % task from es1
task_move_horizontal = TaskMoveHorizontal(); % task from es2 (did at home)
task_move_horizontal_by_prof = TaskHorizontalPos(); % task from es2
task_min_altitude = TaskMinAltitude(); % task from es3
task_min_altitude.name = "min alt";
task_alt_control = TaskAltitudeControl();
task_alt_control.name = "alt ctrl";
task_attitude_control = TaskAttitudeControl();
task_attitude_control.name = "att ctrl";

%move_tool_tasks = {task_tool}; % if we add this as well as lower priority
%task, the vehicle tries to reach the rock, but the two tasks are against
%each other. If we switch priority orders, then the behavior differs,
%vehicle wants to go towards a certain position, but cannot and arm tries
%to push towards the rock, in particular unitl the arm is stretched away
%the vehicle can still move, but once its streched it blocks, arm needs
%vehicle to move towards that direction (higher proority task)
% if we change goal position it may be possible to make two tasks execute
% together -> no contraddiction, however if no constraints of vehicle to
% orientation there may be some problem and unlikely configuration, idea is
% to keep vehicle horizontal, such as task_move_horizontal does
safe_navigation = {task_min_altitude, task_move_horizontal, task_vehicle_pos, task_attitude_control};
landing = {task_alt_control, task_move_horizontal, task_vehicle_pos};
unified_task_list = {task_min_altitude, task_alt_control, task_move_horizontal, ...
                    task_vehicle_pos, task_attitude_control};

% if i add also the task for horizontal orientation the robot maintains
% itself flat. If tool task is higher priority after arm stretches out,
% then the robot starts to go down, even if it goes against vertical task
% (not matching required vertical pos)
% still arm stretches out, to avoid either provide adguate goal position,
% but difficult to know, or create another task that ensure the arm stays
% in a certain circle, or arm never goes below certain position wrt to
% vehicle.

% Actions are registered and selected by name; the task order still defines strict priority within each action.
actionManager = ActionManager(robotModel);

actionManager.addAction("safe navigation", safe_navigation);
actionManager.addAction("landing", landing);
actionManager.addUnifyingTaskList(unified_task_list);
actionManager.setCurrentAction("safe navigation");

% The logger monitors the tasks belonging to the action selected above.
task_set = unified_task_list;


%% Set desired poses
% Positions are in metres and orientations are [roll pitch yaw] in radians. Both poses are expressed in the inertial/world frame.
w_arm_goal_position = [50, 14, -25]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [50 14 -33]';
w_vehicle_goal_orientation = [0, 0, 0];

endFirstAction = 0;

% Store the goals as homogeneous transforms in the shared model.
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Preallocate one column for the initial state plus all integration steps.
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set);

%% Run the closed-loop kinematic simulation
tic

printed = 0;
timePassedFromTransition = 0;

for step = 1:sim.maxSteps
    % Read the newest sensor sample. If none is available, the interface keeps the previous altitude value.
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % Update every task and solve the current strict-priority hierarchy.
    [v_nu, q_dot] = actionManager.computeICAT(robotModel,endFirstAction,printed,timePassedFromTransition);

    % Integrate arm and vehicle velocities, then refresh all transforms.
    sim.step(v_nu, q_dot);

    % Send the state after integration so the viewer displays the new sample.
    unity.send(robotModel);

    % Store state, task references and activations at the current sample.
    logger.update(sim.time, sim.loopCounter);

    % Print a compact status update approximately once per simulated second.
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
        fprintf('position error = %.2f m \n', norm(positionError));
        printed = 0;
    end

    % Busy-wait until one sample period has elapsed when real-time playback is desired. 
    % Remove this call to execute the simulation as fast as possible.
    %SlowdownToRealtime(dt);

    positionError = robotModel.wTv(1:3,4) - w_vehicle_goal_position;
    if norm(positionError) < 0.5 & endFirstAction == 0
        endFirstAction = 1;
    end

    if endFirstAction == 1
        timePassedFromTransition = timePassedFromTransition + dt;
    end
end

%% Review results and release communication resources
logger.plotAll();

% Explicit deletion closes UDP sockets before the script terminates.
delete(unity);
