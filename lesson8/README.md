[< Previous lesson](../lesson7/) -- [**Main Readme**](../README.md)

# Lesson 8 - Testing in the CARLA simulator

In this final lesson, you will run the whole framework from the previous lessons in closed loop inside the CARLA simulator: the simulated world reacts to your vehicle, and your vehicle must react to the world.

Two tools are used for the closed-loop validation:
* [**CARLA**](https://carla.org/) - an open-source autonomous driving simulator. It renders the world via provided map files (and we will use our own Tartu map), simulates the physics and the sensors (lidar, cameras), and feeds them to your nodes through ROS topics.
* **Visual Scenario Editor (VSE)** - a graphical tool for creating and re-playing driving scenarios in CARLA: NPC vehicles and pedestrians with routes and triggers, traffic light sequences and weather. See the [VSE repository](https://github.com/UT-ADL/visual-scenario-editor) and [how to use the editor](https://github.com/UT-ADL/visual-scenario-editor/blob/main/tutorial.md).

You will first verify that your framework can drive in CARLA, then run it through a prepared VSE scenario, and finally design scenarios yourself where your framework fails.

### Expected outcome
* Understanding how the full autonomous driving stack behaves in a closed-loop simulation
* Exploring the limits of the framework you built


## 1. Run your stack in CARLA

The launch file [lesson8.launch](launch/lesson8.launch) connects your nodes from the previous lessons to CARLA. There is no bag playback: the localization comes from the simulator, and the vehicle commands from your `pure_pursuit_follower` steer the car in the simulation.

By default the detected objects and traffic light statuses come from the simulator's ground truth instead of your perception nodes - simulating the lidar and the cameras is very heavy, and running the perception pipeline on them can slow the simulation down to a crawl. Your planner and controller are still the ones driving. If your machine can afford it, you can enable your own perception with `detector:=cluster` (lesson 5 nodes on the simulated lidar) and/or `tfl_detector:=yolo` (lesson 7 nodes on the simulated cameras).

##### Instructions
1. Start the CARLA simulator:
    ```
    $CARLA_ROOT/CarlaUE4.sh -prefernvidia -RenderOffScreen
    ```
2. In another terminal, launch your stack:
    ```
    roslaunch autoware_mini_tutorial lesson8.launch
    ```

##### Validation
* RViz opens with the Tartu map and the ego vehicle placed in the simulated city
* The `Carla image view` panel shows the third-person view of the ego vehicle in the simulated world
* Place a goal on the map - the vehicle drives to it


## 2. Run the demo scenario

A driving scenario adds actors to the otherwise empty world: NPC vehicles and pedestrians that spawn, move and react when triggered, and traffic lights that switch according to the scenario triggers. You will run the prepared demo lap scenario and see whether your framework survives traffic.

When your stack is running, VSE automatically detects your ego vehicle and hands the driving over to it - the scenario provides the destination, the other actors and the evaluation.

##### Instructions
1. With `lesson8.launch` running, start VSE and open the `tartu_demo` map. When VSE first launches, it will ask to select the agent's behavior logic. Navigate to `autoware_mini/nodes/platform/carla/` and select `carla_minimal_agent.py`.
2. Open the scenario (`Scenario` menu -> `Open`): `shared/data/scenarios/tartu_demo_route_simplified.json` from the tutorial folder
3. Press **Play**. Note: if your machine has less than 10 Gb VRAM slowdowns are expected.

##### Validation
* The goal appears in RViz automatically and the vehicle starts driving the demo lap
* NPC vehicles and pedestrians act out the scenario around the ego vehicle
* When the run ends, VSE shows a results window scoring the drive (collisions, red light violations, route completion); the same results are also saved as a text file next to the scenario JSON


## 3. Create three failure cases

Your framework from the previous lessons is a simplified one. Remember all limitations that were discussed through the lessons. In this final task you will demonstrate these limits: create three scenarios where your framework fails.

##### Instructions
1. Copy `tartu_demo_route_simplified.json` (e.g. to `failure_case_1.json`) and modify it in VSE - move, add, retime or reroute actors and triggers until your stack demonstrably fails, while a careful human driver would still manage
2. For every failure case, think of a specific change to the framework that would fix it. You do not need implement the fix. The three cases should have three different proposed fixes.
3. Create a `lesson8/scenarios/` folder in your repository and commit the three scenario JSONs there
4. Fill in the three descriptions below: what happens in the scenario, how your framework fails, and what change to the framework would fix it. Add screenshots if needed.
5. Commit and push everything, and be ready to demonstrate your failure cases at the practice session


##### Failure Case 1: Unecessary Braking

    Target Criterion: CollisionTest

    Scenario Context: In lesson8/scenarios/failure_case_1.json, an actor vehicle is traveling straight along its path behind the ego vehicle. Without any lane cutting or obstacles ahead, the ego vehicle suddenly executes unnecessary, aggressive hard braking.

    Failure Analysis: The ego vehicle's framework generates a false-positive collision constraint or miscalculates path clearance, triggering an emergency stop when no threat exists. Because the ego vehicle stops abruptly, the trailing actor vehicle moving straight behind cannot react in time and rear-ends the ego vehicle. The impact causes the actor vehicle to lose control and swerve wildly across the road.

    Proposed Fix: Improve perception filtering and longitudinal planning validation to eliminate false-positive emergency braking. The planner should require multi-frame verification of obstacle constraints before issuing harsh deceleration commands, while incorporating rear-traffic awareness to avoid inducing rear-end collisions.

##### Failure Case 2: Illegal U-Turn at Merging Lane

    Target Criterion: CollisionTest

    Scenario Context: In lesson8/scenarios/failure_case_2.json, an actor vehicle executes an illegal U-turn across a merging lane directly into the ego vehicle's path, resulting in a crash. A defensive human driver recognizes high-risk, multi-lane merge zones and anticipates potential traffic anomalies—such as lost drivers or illegal turn attempts—by covering the brake and maintaining extra spacing.

    Failure Analysis: The framework operates under a strict ideal-world assumption that all actors follow traffic rules and lane designations. Because it fails to account for non-compliant or out-of-distribution traffic behavior until the offending vehicle is already perpendicular across the lane, the ego vehicle lacks the stopping distance required to prevent a high-speed impact.

    Proposed Fix: Integrate anomaly-aware behavior prediction and risk assessment for complex road geometries like merge lanes. The motion predictor should monitor non-standard vehicle states (e.g., high yaw rates or low speeds in merge zones) and prompt the planner to defensively reduce speed and increase follow distance when anomaly indicators are detected.

##### Failure Case 3: Blind Spot Jaywalking

    Target Criterion: CollisionTest.

    Scenario Context: In lesson8/scenarios/failure_case_3.json, a pedestrian emerges suddenly from behind an occlusion or blind spot and steps into the ego lane at close range. A careful human driver slows down when approaching occluded areas, covers the brake, and leaves additional lateral clearance near potential pedestrian hotspots.

    Failure Analysis: The system evaluates collision risk purely against visible, currently detected objects and lacks spatial reasoning for unobserved or occluded regions. When a pedestrian abruptly emerges from a blind spot, the ego vehicle lacks sufficient headway to execute a smooth emergency stop.

    Proposed Fix: Implement occlusion-aware velocity profiling and pedestrian risk mapping. The motion planner should evaluate visibility fields, lowering the ego speed when passing near large occlusions, parked vehicles, or blind crosswalks to maintain a safe stopping distance for potential hidden hazards.