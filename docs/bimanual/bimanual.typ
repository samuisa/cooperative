// ==========================================
// 1. CONFIGURAZIONI E STILE (Stile con Box)
// ==========================================
#set page(
  paper: "a4",
  margin: (x: 2.5cm, y: 2.5cm),
  numbering: "1",
)

#set text(font: "Linux Libertine", size: 11pt)

// Impostazioni paragrafo
#set par(justify: true, leading: 0.65em)

// Numerazione gerarchica dei capitoli
#set heading(numbering: "1.1.")

// Stile dei Link
#show link: set text(fill: rgb("#000066"))

// --- STILE TITOLI (Con Box) ---

#show heading.where(level: 1): it => {
  v(1em)
  block(
    width: 100%,
    fill: rgb("#2C3E50"), // Blu scuro
    inset: (x: 12pt, y: 10pt),
    radius: 4pt,
    text(size: 16pt, weight: "bold", fill: white)[#it],
  )
  v(0.5em)
}

#show heading.where(level: 2): it => {
  v(1em)
  block(
    width: 100%,
    fill: rgb("#ECF0F1"), // Grigio chiaro
    stroke: (left: 4pt + rgb("#7F8C8D")), // Bordo sinistro spesso
    inset: (x: 10pt, y: 8pt),
    radius: (right: 4pt),
    text(size: 14pt, weight: "bold", fill: rgb("#2C3E50"))[#it],
  )
  v(0.5em)
}

#show heading.where(level: 3): it => {
  v(0.8em)
  block(
    width: 100%,
    fill: rgb("#F8F9F9"), // Grigio chiarissimo
    stroke: (left: 3pt + rgb("#BDC3C7")), // Bordo sinistro medio
    inset: (x: 8pt, y: 6pt),
    radius: (right: 3pt),
    text(size: 12pt, weight: "bold", style: "italic", fill: rgb("#34495E"))[#it],
  )
  v(0.4em)
}

#show heading.where(level: 4): it => {
  v(0.6em)
  block(
    width: 100%,
    fill: none, // Nessun riempimento per non appesantire
    stroke: (left: 2pt + rgb("#95A5A6")), // Bordo sinistro sottile
    inset: (x: 6pt, y: 4pt),
    text(size: 11pt, weight: "bold", fill: rgb("#2C3E50"))[#it],
  )
  v(0.3em)
}


// --- STILE CODICE (Stile LaTeX Listings) ---

#show raw.where(block: true): set text(size: 9pt)

#show raw.where(block: true): it => block(
  width: 100%,
  fill: rgb("#fafafa"),
  stroke: 0.5pt + rgb("#cccccc"),
  radius: 1pt,
  inset: 10pt,
  text(fill: black)[#it],
)

#show raw.where(block: false): it => box(
  fill: rgb("#f5f5f5"),
  stroke: 0.5pt + rgb("#cccccc"), // Aggiunge il bordo grigio
  inset: (x: 4pt, y: 2pt), // Aggiunge respiro anche in verticale
  outset: (y: 3pt), // Evita che il box alteri l'interlinea del paragrafo
  radius: 3pt, // Arrotonda leggermente gli angoli
  text(fill: black)[#it],
)


// --- FRONTESPIZIO / TITOLO ---

#let academic_title(main_title, subtitle: none) = {
  align(center)[
    #v(2em)
    #text(size: 22pt, weight: "bold")[#main_title]

    #if subtitle != none {
      v(1em)
      text(size: 14pt, style: "italic", fill: rgb("#333333"))[#subtitle]
    }

    #v(2em)
    #line(length: 50%, stroke: 0.5pt + rgb("#888888"))
    #v(2em)
  ]
}

#let callout(title, body, color: rgb("#ebf8ff"), border: rgb("#3182ce")) = block(
  width: 100%,
  fill: color,
  stroke: (left: 3pt + border, rest: 0.5pt + border.lighten(60%)),
  inset: 10pt,
  radius: 3pt,
  [
    #text(weight: "bold", fill: border.darken(20%))[#title] \
    #body
  ],
)



= System Kinematics and Reference Frames

The bimanual setup consists of two 7-DOF Franka Emika Panda manipulators controlled as a single 14-DOF kinematic system with joint velocity vector:
$ bold(dot(q)) = vec(bold(dot(q))_L, bold(dot(q))_R) in RR^(14) $

== Base and Tool Transformations (`PandaArm.m` & `BimanualMain.m`)
Each arm shares the same URDF model (`panda.mat`), differing only in its fixed world-to-base transformation $""^w T_b$:
- *Left Arm Base:* Coincides with the world frame ($""^w T_(b,L) = I_4$).
- *Right Arm Base:* Translated by $[1.06, -0.01, 0]^T$ m and rotated by $theta = pi$ around the world $z$-axis:
  $
    ""^w T_(b,R) = mat(
      cos pi, -sin pi, 0, 1.06;
      sin pi, cos pi, 0, -0.01;
      0, 0, 1, 0;
      0, 0, 0, 1
    )
  $
- *End-Effector to Tool Frame ($""^e T_t$):* The tool frame is placed $0.2104$ m ($21.04$ cm) along the end-effector $z$-axis (`panda_link7`) and rotated by $theta_t = -44.9949 degree$ around $z_e$ so that the gripper fingers align with the local axes:

```matlab
% PandaArm.m: fixed end-effector-to-tool transformation
theta = deg2rad(-44.9949);
obj.eTt = [cos(theta) -sin(theta) 0  0;
           sin(theta)  cos(theta) 0  0;
               0           0      1  0.2104;
               0           0      0  1];
```

== Tool Jacobian Shifting
Since `geometricJacobian` returns the end-effector Jacobian $""^b J_e$ at `panda_link7` (ordered as `[angular; linear]`), we first rotate it into the world frame and then shift the reference point from the flange to the tool tip using the rigid-body velocity twist operator $S_(t e)$:
$
  ""^w J_t = underbrace(mat(I_3, 0_(3 times 3); -[""^w R_e \, ""^e p_t]_times, I_3), S_(t e)) mat(""^w R_b, 0_(3 times 3); 0_(3 times 3), ""^w R_b) ""^b J_e
$

= Phase 1: Reaching the Object & Grasp Geometry

== Grasp Goal Calculation (`BimanualMain.m`)
Given the initial object center $""^w O_o = [0.5, 0, 0.59]^T$ m and object length $l_"obj" = 0.12$ m, the two grasping points lie on opposite sides along the world $x$-axis:
$
  p_(g,L) = ""^w O_o - vec(l_"obj"/2, 0, 0) = vec(0.44, 0, 0.59) "m", quad p_(g,R) = ""^w O_o + vec(l_"obj"/2, 0, 0) = vec(0.56, 0, 0.59) "m"
$

To achieve the symmetric grasping configuration, each tool frame must be rotated by $psi = 30 degree$ around its *own local $y$-axis* starting from its initial downward-pointing orientation $""^w R_(t_i, "init")$:

```matlab
% BimanualMain.m: computing grasping goals from object pose and initial tool frames
l_obj = 0.12;
psi = deg2rad(30);

toolGoalPositionLeftArm  = objectPosition - [l_obj/2; 0; 0]; % [0.44; 0; 0.59]
toolGoalPositionRightArm = objectPosition + [l_obj/2; 0; 0]; % [0.56; 0; 0.59]

toolGoalOrientationLeftArm  = leftArm.wTt(1:3,1:3)  * RPYToRotationMatrix(0, psi, 0);
toolGoalOrientationRightArm = rightArm.wTt(1:3,1:3) * RPYToRotationMatrix(0, psi, 0);

leftArm.setGoal(objectPosition, objectOrientation, toolGoalPositionLeftArm, toolGoalOrientationLeftArm);
rightArm.setGoal(objectPosition, objectOrientation, toolGoalPositionRightArm, toolGoalOrientationRightArm);
```

#callout(
  "Why the Initial Orientation Setup Caused Collisions",
  [
    Passing `RPYToRotationMatrix(0, psi, 0)` directly into `setGoal` defines the rotation relative to the *world frame* (where $+z$ points upward) rather than post-multiplying the *initial tool frame* (where $+z$ points downward). Because the tool origin is $21.04$ cm ahead of the wrist flange, commanding an upward-pointing tool forced both grippers to flip $180 degree$ upside down while converging toward $x = 0.50$ m, causing the wrists to sweep through the center and collide. Post-multiplying $""^w R_(t,"init")$ tilts the grippers inward by $30 degree$ while keeping the wrists safely separated at $x approx 0.335$ m and $x approx 0.665$ m.
  ],
  color: rgb("#fff5f5"),
  border: rgb("#c53030"),
)

= Task-Priority Hierarchy & Safety Tasks

In the iCAT (inequality-constrained Task-Priority) framework, tasks are stacked in strict priority order. Lower-priority tasks are projected into the null space of higher-priority tasks, modulated by diagonal activation matrices $A in [0, 1]^(m times m)$.

== Minimum Table Altitude (`TaskMinAltitude.m`)
A scalar ($1 times 14$) inequality task keeping the tool altitude $z_t = ""^w T_t (3, 4)$ above $z_"min" = 0.15$ m. Row 6 of $""^w J_t$ extracts the linear vertical velocity $dot(z)_t$ in world coordinates:

```matlab
classdef TaskMinAltitude < Task
    properties
        robot
        armID = ""
        feedGain = 0.5
        altMin = 0.15
        altThreshold = 0.18
    end
    methods
        function obj = TaskMinAltitude(robot, armID)
            obj.robot = robot;
            obj.armID = string(armID);
            obj.taskName = obj.armID + " min altitude";
        end

        function updateReference(obj)
            currentAlt = obj.robot.wTt(3, 4);
            obj.xdotbar = obj.feedGain * (obj.altMin - currentAlt);
        end

        function updateJacobian(obj)
            if obj.armID == "left"
                obj.J = [obj.robot.wJt(6, :), zeros(1, 7)];
            else
                obj.J = [zeros(1, 7), obj.robot.wJt(6, :)];
            end
        end

        function updateActivation(obj)
            currentAlt = obj.robot.wTt(3, 4);
            obj.A = DecreasingBellShapedFunction(obj.altMin, obj.altThreshold, 0, 1, currentAlt);
        end
    end
end
```

== Joint Limits Avoidance (`TaskJointLimits.m`)
A 7-dimensional ($7 times 14$) inequality task enforcing $q_(i,"min") <= q_i <= q_(i,"max")$ for each joint. Crucially, the activation matrix $A$ must be *diagonal* (`diag(a)`) so that only the single joint approaching a limit is restricted without freezing the remaining 6 DOFs:

```matlab
classdef TaskJointLimits < Task
    properties
        robot
        armID = ""
        activationBuffer = 0.05 % [rad] safety interval before jlmin/jlmax
        feedbackGain = 1
    end
    methods
        function obj = TaskJointLimits(robot, armID)
            obj.robot = robot;
            obj.armID = string(armID);
            obj.taskName = obj.armID + " joint limits";
        end

        function updateReference(obj)
            q = obj.robot.q;
            lowerError = max((obj.robot.jlmin + obj.activationBuffer) - q, 0);
            upperError = min((obj.robot.jlmax - obj.activationBuffer) - q, 0);
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
            a = zeros(7, 1);
            for i = 1:7
                aLow  = DecreasingBellShapedFunction(obj.robot.jlmin(i), obj.robot.jlmin(i) + obj.activationBuffer, 0, 1, q(i));
                aHigh = IncreasingBellShapedFunction(obj.robot.jlmax(i) - obj.activationBuffer, obj.robot.jlmax(i), 0, 1, q(i));
                a(i) = max(aLow, aHigh);
            end
            obj.A = diag(a);
        end
    end
end
```

== Comparison: `TaskManipulability` vs. `TaskJointLimits`
#table(
  columns: (1.2fr, 1.8fr, 1.8fr),
  inset: 7pt,
  fill: (col, row) => if row == 0 { rgb("#edf2f7") } else { white },
  stroke: 0.5pt + rgb("#cbd5e0"),
  [*Property*], [*`TaskManipulability`*], [*`TaskJointLimits`*],
  [Control Space], [Task-space scalar index (Yoshikawa $w(q)$)], [Joint-space coordinates ($q_1, dots, q_7$)],
  [Dimension], [$1 times 14$ (scalar)], [$7 times 14$ (7 independent rows)],
  [Jacobian $J$],
  [Numerical gradient $partial w / partial q$ ($1 times 7$)],
  [Identity matrix $I_7$ ($partial q / partial q$)],

  [Activation $A$], [Scalar in $[0, 1]$ (lower bound only)], [Diagonal $7 times 7$ matrix (lower & upper bounds)],
  [Mission Scope], [Active in Phase 1 (`go to object`)], [Active in Phases 1 & 2 (`go to object`, `move object`)],
)

= Phase 2: Cooperative Rigid-Body Transportation

== Computing Constant Grasp Transforms
`PandaArm.wTo` stores the initial object pose and does not update automatically during motion. At the exact instant both tools reach their grasping poses (transition from Phase 1 to Phase 2), we compute the constant object pose expressed in each tool frame:
$ ""^(t_L) T_o = (""^w T_(t_L))^(-1) \, ""^w T_o, quad ""^(t_R) T_o = (""^w T_(t_R))^(-1) \, ""^w T_o $
During Phase 2, the current object pose is reconstructed via forward kinematics as $""^w T_o (t) = ""^w T_(t_L)(t) \, ""^(t_L) T_o$.

== Rigid Grasp Constraint (`TaskRigidConstraint.m`)
To carry the object as a rigid body without internal stress or slippage, the object frame attached to the left tool and the one attached to the right tool must share the exact same world Cartesian velocity ($""^w v_(o,L) = ""^w v_(o,R)$):
$
  ""^w J_(o,L) bold(dot(q))_L - ""^w J_(o,R) bold(dot(q))_R = bold(0)_(6 times 1) quad arrow.r.double quad J_"rigid" = mat(""^w J_(o,L), -""^w J_(o,R)) in RR^(6 times 14)
$
Per specification, the activation of the rigid constraint is *binary* ($A = I_6$, without smooth transitions):

```matlab
classdef TaskRigidConstraint < Task
    properties
        leftArm
        rightArm
        tLo = eye(4)
        tRo = eye(4)
        isActive = true
    end
    methods
        function obj = TaskRigidConstraint(leftArm, rightArm, taskName)
            obj.leftArm = leftArm;
            obj.rightArm = rightArm;
            obj.taskName = string(taskName);
        end

        function computeGraspTransforms(obj)
            obj.tLo = obj.leftArm.wTt \ obj.leftArm.wTo;
            obj.tRo = obj.rightArm.wTt \ obj.rightArm.wTo;
        end

        function updateReference(obj)
            obj.xdotbar = zeros(6, 1);
        end

        function updateJacobian(obj)
            StoL = [eye(3), zeros(3); -skew(obj.leftArm.wTt(1:3,1:3) * obj.tLo(1:3,4)), eye(3)];
            wJoL = StoL * obj.leftArm.wJt;

            StoR = [eye(3), zeros(3); -skew(obj.rightArm.wTt(1:3,1:3) * obj.tRo(1:3,4)), eye(3)];
            wJoR = StoR * obj.rightArm.wJt;

            obj.J = [wJoL, -wJoR];
        end

        function updateActivation(obj)
            obj.A = eye(6) * obj.isActive;
        end
    end
end
```

== Object Transportation (`TaskObject.m`)
Placed at a lower priority than `TaskRigidConstraint`, `TaskObject` drives the grasped object frame toward the target pose $""^w T_(o g)$ (`[0.65; -0.35; 0.28]`). Because iCAT projects `TaskObject` into the null space of `TaskRigidConstraint`, commanding the left arm automatically drags the right arm along the rigid-body trajectory:

```matlab
classdef TaskObject < Task
    properties
        robot
        tTo = eye(4)
        feedbackGain = 1
        maxAngularVelocity = 0.3
        maxLinearVelocity = 0.3
    end
    methods
        function obj = TaskObject(robot, taskName)
            obj.robot = robot;
            obj.taskName = string(taskName);
        end

        function computeGraspTransform(obj)
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
            Sto = [eye(3), zeros(3); -skew(obj.robot.wTt(1:3,1:3) * obj.tTo(1:3,4)), eye(3)];
            wJo = Sto * obj.robot.wJt;
            obj.J = [wJo, zeros(6, 7)];
        end

        function updateActivation(obj)
            obj.A = eye(6);
        end
    end
end
```

= Phase 3: Mission Completion & State Machine

== Zero-Velocity Stop Task (`TaskStop.m`)
Once the object reaches $""^w T_(o g)$, all tasks are deactivated except the minimum altitude safety task and a full-system 14-DOF joint-velocity stop task ($I_(14) bold(dot(q)) = bold(0)_(14 times 1)$):

```matlab
classdef TaskStop < Task
    methods
        function obj = TaskStop(taskName)
            obj.taskName = string(taskName);
        end
        function updateReference(obj),  obj.xdotbar = zeros(14, 1); end
        function updateJacobian(obj),   obj.J = eye(14);            end
        function updateActivation(obj), obj.A = eye(14);            end
    end
end
```

== Action Stacks & Closed-Loop State Machine (`BimanualMain.m`)
The three mission phases are registered in `ActionManager` in strict descending priority order, and transitions are evaluated *after* `computeICAT()` so that Cartesian errors are freshly updated:

```matlab
% Action stacks (highest priority first)
goToObjectTasks = {jointLimitsLeft, jointLimitsRight, minAltLeft, minAltRight, ...
                   leftManipulabilityTask, rightManipulabilityTask, leftToolTask, rightToolTask};
moveObjectTasks = {jointLimitsLeft, jointLimitsRight, minAltLeft, minAltRight, ...
                   rigidConstraintTask, objectTask};
stopTasks       = {minAltLeft, minAltRight, stopTask};

actionManager.addAction("go to object", goToObjectTasks);
actionManager.addAction("move object", moveObjectTasks);
actionManager.addAction("stop", stopTasks);

% Inside the closed-loop simulation step (after q_dot = actionManager.computeICAT()):
if phase == 1
    leftReached  = norm(leftArm.positionError)  < posTol && norm(leftArm.orientationError)  < oriTol;
    rightReached = norm(rightArm.positionError) < posTol && norm(rightArm.orientationError) < oriTol;
    if leftReached && rightReached
        rigidConstraintTask.computeGraspTransforms();
        objectTask.computeGraspTransform();
        actionManager.setCurrentAction("move object");
        phase = 2;
    end
elseif phase == 2
    wTo_current = leftArm.wTt * objectTask.tTo;
    [objOriErr, objPosErr] = CartesianError(leftArm.wTog, wTo_current);
    if norm(objPosErr) < posTol && norm(objOriErr) < oriTol
        actionManager.setCurrentAction("stop");
        phase = 3;
    end
end
```

#callout(
  "Key Implementation Takeaways",
  [
    1. *Constructor Arguments & `nargin`:* `ActionManager` invokes `updateReference()`, `updateJacobian()`, and `updateActivation()` with zero arguments. Each task class must therefore store its target `PandaArm` handle and `armID` (`"left"` or `"right"`) during construction. MATLAB's `nargin` checks the number of input arguments passed to make parameters like `taskName` or thresholds optional.
    2. *Avoiding Single-Joint Lockup:* In `TaskJointLimits`, using a scalar activation `a * eye(7)` freezes the entire arm whenever a single joint enters the buffer zone (which stalled the right arm $10$ cm short of `wTog`). Using `diag(a)` with a tight buffer (`0.05` rad) isolates only the constrained joint and lets the remaining 6 DOFs complete the motion.
  ],
)
