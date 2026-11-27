
function BimanualMain()
%BIMANUALMAIN Assemble and run the two-Panda task-priority simulation.
%
% The control vector is q_dot = [q_dot_left; q_dot_right]. At every sample the active task hierarchy computes this fourteen-dimensional reference, BimanualSim integrates it, UDPInterface sends the resulting state or command, and SimulationLogger stores the signals required for analysis.
%% Make exercise dependencies visible to MATLAB
% if you have issues, clear the paths from Home -> Set Path to avoid
% ambiguities with other scripts
addpath('./tools');
addpath('./icat');
addpath('./tpik');

%% Configure the simulation
dt = 0.005;
endTime = 20;
realRobot = false;

% Each PandaArm loads the common mechanical model from panda.mat. The two instances differ only through their fixed world-to-base transforms.
wTbLeft = eye(4);
theta = pi;
wTbRight = ...
[cos(theta) -sin(theta) 0  1.06;
 sin(theta) cos(theta)  0 -0.01;
     0          0       1   0;
     0          0       0   1];
leftArm = PandaArm(wTbLeft);
rightArm = PandaArm(wTbRight);
bimanualSystem = BimanualSim(dt, leftArm, rightArm, endTime);

%% Define the object and individual tool goals
objectPosition = [0.5; 0; 0.59];
objectOrientation = RPYToRotationMatrix(0, 0, 0);

% TODO: derive distinct left and right tool goals from the object pose and the desired grasp transforms.
l_obj = 0.12;
psi = deg2rad(30);

% Posizioni dei punti di presa rispetto al centro dell'oggetto
toolGoalPositionLeftArm  = objectPosition - [l_obj/2; 0; 0]; % [0.44; 0; 0.59]
toolGoalPositionRightArm = objectPosition + [l_obj/2; 0; 0]; % [0.56; 0; 0.59]

% Orientamento goal: rotazione di +30° attorno all'asse y locale del tool frame iniziale
wRt_left_init  = leftArm.wTt(1:3, 1:3);
wRt_right_init = rightArm.wTt(1:3, 1:3);

toolGoalOrientationLeftArm  = wRt_left_init  * RPYToRotationMatrix(0, psi, 0);
toolGoalOrientationRightArm = wRt_right_init * RPYToRotationMatrix(0, psi, 0);

leftArm.setGoal(objectPosition, objectOrientation, toolGoalPositionLeftArm, toolGoalOrientationLeftArm);
rightArm.setGoal(objectPosition, objectOrientation, toolGoalPositionRightArm, toolGoalOrientationRightArm);

% The cooperative-motion object goal is stored now for tasks introduced in the following exercise phases.
wTog = [RPYToRotationMatrix(0, 0, 0) [0.65; -0.35; 0.15]; 0 0 0 1];
leftArm.setObjectGoal(wTog);
rightArm.setObjectGoal(wTog);

%% Define named actions as strict-priority task stacks

minimumManipulability = 0.06;
manipulabilityActivationThreshold = 0.07;

% phase 1, reach and grasp object
leftManipulabilityTask      = TaskManipulability(leftArm, "left", "left manipulability", minimumManipulability, manipulabilityActivationThreshold);
rightManipulabilityTask     = TaskManipulability(rightArm, "right", "right manipulability", minimumManipulability, manipulabilityActivationThreshold);
leftToolTask                = TaskTool(leftArm, "left", "left tool");
rightToolTask               = TaskTool(rightArm, "right", "right tool");

goToObjectTasks             = {leftManipulabilityTask, rightManipulabilityTask, leftToolTask, rightToolTask};

% phase 2, move object after grasping
minAltLeft                  = TaskMinAltitude(leftArm, "left");
minAltRight                 = TaskMinAltitude(rightArm, "right");

rigidConstraintTask         = TaskRigidConstraint(leftArm, rightArm, "rigid constraint");
% leftObjectTask                  = TaskObject(leftArm, "move object");
% rightObjectTask                  = TaskObject(rightArm, "move object");

jointLimitsLeft             = TaskJointLimits(leftArm, "left");
jointLimitsRight            = TaskJointLimits(rightArm, "right");

%moveObjectTasks            = {jointLimitsLeft, jointLimitsRight, minAltLeft, minAltRight, rigidConstraintTask, objectTask};
moveObjectTasks             = {jointLimitsLeft, jointLimitsRight, minAltLeft, minAltRight, rigidConstraintTask, leftToolTask, rightToolTask};

% phase 3, stop
stopTask                    = TaskStop("stop motion");
stopTasks                   = {minAltLeft, minAltRight, stopTask};

% register actions
actionManager = ActionManager(bimanualSystem);
actionManager.addAction("go to object", goToObjectTasks);
actionManager.addAction("move object", moveObjectTasks);
actionManager.addAction("stop", stopTasks);
actionManager.setCurrentAction("go to object");


%% Initialize communication and logging
robotUDP = UDPInterface(realRobot);
logger = SimulationLogger(bimanualSystem, goToObjectTasks);

%% Run the closed-loop kinematic simulation
phase = 1; % 1: go to object, 2: move object, 3: stop
posTol = 1e-3; % 1 mm
oriTol = 1e-2; % ~0.57 deg

tic
for step = 1:bimanualSystem.maxSteps
    % In real-robot mode, replace the simulated configuration with the latest measurements before computing the new reference.
    [q_left, q_right] = robotUDP.receive(bimanualSystem.time);
    if realRobot
        bimanualSystem.leftArm.q = q_left;
        bimanualSystem.rightArm.q = q_right;
        bimanualSystem.updateKinematics();
    end

    % Update the active tasks and solve their strict-priority hierarchy.
    q_dot = actionManager.computeICAT();

    if phase == 1
        leftReached  = norm(leftArm.positionError)  < posTol && norm(leftArm.orientationError)  < oriTol;
        rightReached = norm(rightArm.positionError) < posTol && norm(rightArm.orientationError) < oriTol;

        if leftReached && rightReached
            rigidConstraintTask.computeGraspTransforms();

            tTo_left  = leftArm.wTt \ leftArm.wTo;
            tTo_right = rightArm.wTt \ rightArm.wTo;

            wTg_left_move  = wTog / tTo_left;
            wTg_right_move = wTog / tTo_right;

            leftArm.setGoal(wTog(1:3, 4), wTog(1:3, 1:3), wTg_left_move(1:3, 4), wTg_left_move(1:3, 1:3));
            rightArm.setGoal(wTog(1:3, 4), wTog(1:3, 1:3), wTg_right_move(1:3, 4), wTg_right_move(1:3, 1:3));

            % Libera l'orientamento durante il trasporto
            leftToolTask.positionOnly = true;
            rightToolTask.positionOnly = true;

            actionManager.setCurrentAction("move object");
            phase = 2;
            
        end

    elseif phase == 2
        wTo_current = leftArm.wTt * tTo_left;
        [objOriErr, objPosErr] = CartesianError(leftArm.wTog, wTo_current);

        if mod(step, round(1/dt)) == 0
            fprintf('object position: x %.2f, y %.2f, z %.2f\n', wTo_current(1, 4), wTo_current(2, 4), wTo_current(3, 4));
        end

        if norm(objPosErr) < posTol
            actionManager.setCurrentAction("stop");
            phase = 3;
        end
    end

    % Integrate both arms and refresh their transforms and Jacobians.
    bimanualSystem.step(q_dot);

    % Send either velocity commands to the physical robots or joint positions to PyBullet, then store the completed sample.
    robotUDP.send(bimanualSystem.time, bimanualSystem);
    logger.update(bimanualSystem.time, bimanualSystem.loopCounter);

    if mod(step, round(1/dt)) == 0
        fprintf('t = %.2f s | phase: %d\n', bimanualSystem.time, phase);
        % fprintf('left tool orientation: (%.2f, %.2f, %.2f) \n', leftarm.wTt())
        % disp(leftArm.wTt(1:3,1:3));
        % disp(rightArm.wTt(1:3,1:3));
    end
    SlowdownToRealtime(dt);
end

disp([minAltLeft.A, minAltRight.A]);                 % sarà [0, 0]
disp(find(diag(jointLimitsLeft.A) > 0)');            % giunti sinistri nel buffer 0.05 rad
disp(find(diag(jointLimitsRight.A) > 0)');           % giunti destri nel buffer 0.05 rad
disp([leftArm.computeManipulability(1e-6,1e-4,1e-4), rightArm.computeManipulability(1e-6,1e-4,1e-4)]);

%% Review results and release communication resources
logger.plotAll();
delete(robotUDP);
end
