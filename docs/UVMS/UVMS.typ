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

= Exercise 1

== Requirements

*Goal*: Implement a vehicle position control task, and test that the vehicle reaches the required position. Add the proper Jacobian variable, its activation function, and the desired reference rate.

To test that the task is working properly

- Initialize the vehicle far away from the seafloor. An example position could be: \
  eta = [10.5 35.5 -36 0 0 pi/2]  (x,y,z,roll,pitch,yaw)
- Give a target position that is also sufficiently away from the seafloor, e.g.,  \
  w_vehicle_goal_position  = [10.5 37.5 -38]

== Solution

- `TaskVehiclePosition.m`: \

  - The `updateReference` function computes the desired vehicle velocity based exclusively on the linear position error, as the exercise does not require orientation control.

  - The `updateJacobian`function maps the vehicle's linear velocities to the world frame using `robot.wTv(1:3, 1:3)`. The first seven columns (arm joints) and the last three columns (vehicle orientation) are set to zero because those degrees of freedom are ignored by this task.

  - The `updateActivation` function enables control for these three linear axes by returning a 3x3 identity matrix.

- `RobustMain.m`

  - The custom task is instantiated and prepared for the controller:
    ```matlab
    task_vehicle = TaskVehiclePosition();
    ```
  - The action is then registered and selected as the active priority stack:
    ```matlab
    actionManager.addAction("move_vehicle", move_vehicle_tasks);
    actionManager.setCurrentAction("move_vehicle");
    ```
  - Finally, the task stack is assigned to the logger for runtime monitoring:

    ```matlab
        task_set = move_vehicle_tasks;
    ```

= Exercise 2

== Requirements

*Goal*: Implement a horizontal attitude control task, and test that the vehicle reaches the required orientation. Add the proper Jacobian variable, its activation function, and the desired reference rate.

To test that the task is working properly

- Initialize the vehicle far away from the seafloor. An example position could be: \
  eta = [10.5 35.5 -36 -pi/3 pi/3 pi/2]  (x,y,z,roll,pitch,yaw)
- Require that the norm of the misalignment between the vehicle's z vector and the absolute z-axis is less than 0.2

== Solution

- `TaskVehicleMisalignment.m`:
  - The `updateReference` function computes the desired angular velocity to correct the misalignment, obtained through the cross product between the vehicle's z-axis and the absolute z-axis.
  - The `updateJacobian` function places the rotation matrix `wTv(1:3, 1:3)` in the last three columns of the Jacobian. This maps the correction exclusively to the vehicle's angular velocities, ignoring the arm joints (first 7 columns) and the linear velocities (columns 8-10).
  - The `updateActivation` function handles the inequality: it returns `obj.A = a * eye(3)`, where `a` is set to 1 to fully activate the correction if the error exceeds the 0.2 threshold, and 0 to disable the task when the attitude is within safety limits.

= Exercise 3

== Requirements

Initialize the vehicle far away from the seafloor. An example position could be: \
[10.5 35.5 -36 0 0 pi/2]  (x,y,z,roll,pitch,yaw)

Goal: Implement a vehicle altitude control task, adding the proper Jacobian variable, its activation function, and the desired reference rate.
Test the functioning of the position/attitude vehicle control tasks together with the minimum altitude in two situations:
1) one where the desired position is too close to seafloor, for example
[12.2025 37.3748 -39.8860]
2) one where the desired position is instead far away from the seafloor, for example
[10.5 37.5 -38]

== Solution

- `TaskVehicleAltitude.m`:
  - The `updateReference` function computes the desired vertical velocity to maintain a safe altitude
    above the seafloor, based on the current depth and the minimum allowed depth.
    ```matlab
    err_w = obj.altMin - robot.eta(3);
    obj.xdotbar = obj.feedGain * err_w;
    ```
  - The `updateJacobian` function maps the vertical velocity control to the vehicle's linear velocities,
    specifically affecting the z-axis (vertical direction).
    ```matlab
    obj.J = [zeros(3, 7), wRv, zeros(3, 3)];
    ```
  - The `updateActivation` function activates the task when the vehicle is below the minimum altitude threshold,
    returning an identity matrix for full control, and deactivates it when above the threshold.
    ```matlab
    actAlt = robot.eta(3);

    if actAlt < obj.altMin
        a = 1;
    else 
        a = 0;
    end

    obj.A = a * eye(3);
    ```

= Exercise 4

== Requirements

*Exercise 4 Goal*: implement *two actions*, *extending the ActionManager* to manage transition between actions.
Modify the class to associate a name to the action in the addAction, and to select the action with the same name using setCurrentAction.
*Add an API* to add the unifying task list (computed manually). When the computeICAT is called, automatically compute
the activation functions to manage the transition between the current and previous action (hint: the function ismember can be very useful).
To this aim, the ActionManager should keep track of the time since the new action was scheduled (modify as necessary).

To test, implement the following actions:
- Action “safe navigation” includes minimum altitude, horizontal attitude, and vehicle position and attitude control
- Action “landing” includes altitude control (to zero), horizontal attitude, and vehicle position.

Start by moving from an initial point is close to the seafloor, e.g. \
[48.5 11.5 -33] \
And move the vehicle towards a position such as \
[50 -12.5 -33] \
Then land on the seafloor. Include the appropriate transition between the two actions.
Show that the minimum altitude gets activated during the safe navigation (eventually change the threshold values).




== Solution
