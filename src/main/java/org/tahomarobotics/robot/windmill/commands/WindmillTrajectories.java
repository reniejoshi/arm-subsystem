/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.windmill.WindmillState;
import org.tahomarobotics.robot.windmill.WindmillTrajectory;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import static org.tahomarobotics.robot.windmill.WindmillConstants.*;
import static org.tahomarobotics.robot.RobotConfiguration.*;

public class WindmillTrajectories {

    private static final Map<Pair<TrajectoryState, TrajectoryState>, WindmillTrajectory> trajectories = new HashMap<>();

    public static Optional<WindmillTrajectory> getTrajectory(TrajectoryState from, TrajectoryState to) {
        WindmillTrajectory traj = trajectories.get(Pair.of(from, to));
        return Optional.ofNullable(traj);
    }

    static {
        WindmillState collectLift = FEATURE_ALGAE_END_EFFECTOR ?
            createWindmillState(TrajectoryState.CORAL_COLLECT.elev, Units.degreesToRadians(240.0)) :
            createWindmillState(TrajectoryState.CORAL_COLLECT.elev + Units.inchesToMeters(12.931), TrajectoryState.CORAL_COLLECT.arm);

        // Algae
        create(TrajectoryState.CORAL_COLLECT, TrajectoryState.HIGH_DESCORE, new WindmillState[]{collectLift});
        create(TrajectoryState.CORAL_COLLECT, TrajectoryState.LOW_DESCORE, new WindmillState[]{collectLift});
        create(TrajectoryState.HIGH_DESCORE, TrajectoryState.CORAL_COLLECT, new WindmillState[]{collectLift});
        create(TrajectoryState.LOW_DESCORE, TrajectoryState.CORAL_COLLECT, new WindmillState[]{collectLift});
        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.HIGH_DESCORE, SMALL_PULLBACK, TrajectoryState.LOW_DESCORE);
        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.LOW_DESCORE, SMALL_PULLBACK, TrajectoryState.HIGH_DESCORE);

        create(TrajectoryState.STOW, TrajectoryState.LOW_DESCORE);
        create(TrajectoryState.STOW, TrajectoryState.HIGH_DESCORE);
        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.LOW_DESCORE, EXTRA_LARGE_PULLBACK, TrajectoryState.STOW);
        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.HIGH_DESCORE, EXTRA_LARGE_PULLBACK, TrajectoryState.STOW);

        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.HIGH_DESCORE, SMALL_PULLBACK, TrajectoryState.ALGAE_PRESCORE);
        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.LOW_DESCORE, SMALL_PULLBACK, TrajectoryState.ALGAE_PRESCORE);

        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.STOW, TrajectoryState.ALGAE_PRESCORE);
        create(TrajectoryState.STOW, TrajectoryState.ALGAE_COLLECT);

        create(TrajectoryState.ALGAE_SCORE, TrajectoryState.STOW);
        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.ALGAE_PRESCORE, TrajectoryState.STOW);
        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.ALGAE_COLLECT, TrajectoryState.STOW);
        create(WindmillTrajectory.WindmillConstraints.ALGAE_THROW_CONSTRAINTS, TrajectoryState.ALGAE_PRESCORE, TrajectoryState.ALGAE_SCORE);
        create(WindmillTrajectory.WindmillConstraints.ALGAE_CONSTRAINTS, TrajectoryState.STOW, TrajectoryState.ALGAE_PROCESSOR);
        create(TrajectoryState.ALGAE_PROCESSOR, TrajectoryState.CORAL_COLLECT);

        // Algae - Coral
        create(TrajectoryState.L2, SMALL_PULLBACK, TrajectoryState.HIGH_DESCORE);
        create(TrajectoryState.L2, TrajectoryState.LOW_DESCORE);
        create(TrajectoryState.L3, TrajectoryState.HIGH_DESCORE);
        create(TrajectoryState.L3, STANDARD_PULLBACK, TrajectoryState.LOW_DESCORE);
        create(TrajectoryState.L4, LARGE_PULLBACK, TrajectoryState.HIGH_DESCORE);
        create(TrajectoryState.L4, LARGE_PULLBACK, TrajectoryState.LOW_DESCORE);

        create(TrajectoryState.HIGH_DESCORE, SMALL_PULLBACK, TrajectoryState.L2);
        create(TrajectoryState.LOW_DESCORE, TrajectoryState.L2);
        create(TrajectoryState.HIGH_DESCORE, TrajectoryState.L3);
        create(TrajectoryState.LOW_DESCORE, STANDARD_PULLBACK, TrajectoryState.L3);
        create(TrajectoryState.HIGH_DESCORE, LARGE_PULLBACK, TrajectoryState.L4);
        create(TrajectoryState.LOW_DESCORE, LARGE_PULLBACK, TrajectoryState.L4);

        // Coral
        create(TrajectoryState.CORAL_COLLECT, TrajectoryState.L2, new WindmillState[]{collectLift});
        create(TrajectoryState.L2, TrajectoryState.CORAL_COLLECT, new WindmillState[]{collectLift});

        create(TrajectoryState.CORAL_COLLECT, TrajectoryState.L1, new WindmillState[]{collectLift});
        create(TrajectoryState.L1, TrajectoryState.CORAL_COLLECT, new WindmillState[]{collectLift});

        create(TrajectoryState.STOW, TrajectoryState.CORAL_COLLECT, new WindmillState[]{collectLift});
        create(TrajectoryState.CORAL_COLLECT, TrajectoryState.STOW, new WindmillState[]{collectLift});

        create(TrajectoryState.CORAL_COLLECT, TrajectoryState.L3, new WindmillState[]{collectLift});
        create(TrajectoryState.L3, TrajectoryState.CORAL_COLLECT, new WindmillState[]{collectLift});

        create(TrajectoryState.CORAL_COLLECT, TrajectoryState.L4, new WindmillState[]{collectLift});
        create(TrajectoryState.L4, TrajectoryState.CORAL_COLLECT, new WindmillState[]{collectLift});

        create(TrajectoryState.L2, LARGE_PULLBACK, TrajectoryState.L3);
        create(TrajectoryState.L2, LARGE_PULLBACK, TrajectoryState.L4);
        create(TrajectoryState.L3, LARGE_PULLBACK, TrajectoryState.L2);
        create(TrajectoryState.L3, LARGE_PULLBACK, TrajectoryState.L4);
        create(TrajectoryState.L4, LARGE_PULLBACK, TrajectoryState.L2);
        create(TrajectoryState.L4, LARGE_PULLBACK, TrajectoryState.L3);

        create(TrajectoryState.L1, TrajectoryState.L2);
        create(TrajectoryState.L1, TrajectoryState.L3);
        create(TrajectoryState.L1, TrajectoryState.L4);

        create(TrajectoryState.L2, TrajectoryState.L1);
        create(TrajectoryState.L3, TrajectoryState.L1);
        create(TrajectoryState.L4, TrajectoryState.L1);

        create(TrajectoryState.START, TrajectoryState.L4);

        create(TrajectoryState.STOW, TrajectoryState.L1);
        create(TrajectoryState.STOW, TrajectoryState.L2);
        create(TrajectoryState.STOW, TrajectoryState.L3);
        create(TrajectoryState.STOW, TrajectoryState.L4);

        create(TrajectoryState.L1, TrajectoryState.STOW);
        create(TrajectoryState.L2, TrajectoryState.STOW);
        create(TrajectoryState.L3, TrajectoryState.STOW);
        create(TrajectoryState.L4, TrajectoryState.STOW);
    }

    private static WindmillState createWindmillState(double elev, double arm) {
        return new WindmillState(
            0,
            new WindmillState.ElevatorState(elev, 0, 0),
            new WindmillState.ArmState(arm, 0, 0)
        );
    }

    private static void create(TrajectoryState start, double pullback, TrajectoryState end) {
        double elev = (start.elev + end.elev) / 2;
        double arm = (start.arm + end.arm) / 2 - pullback;
        var pos = new WindmillState(
            0,
            new WindmillState.ElevatorState(elev, 0, 0),
            new WindmillState.ArmState(arm, 0, 0)
        );
        create(start, end, new WindmillState[]{pos});
    }

    private static void create(TrajectoryState start, TrajectoryState end, WindmillState midPositions[]) {
        WindmillState[] states = new WindmillState[2 + midPositions.length];
        states[0] = start.state;
        states[states.length - 1] = end.state;
        for (int i = 0; i < midPositions.length; i++) {
            states[i + 1] = midPositions[i];
        }
        String name = start.name() + "_TO_" + end.name();
        try {
            trajectories.put(new Pair<>(start, end), new WindmillTrajectory(name, states));
        } catch (Exception e) {
            System.err.println("Trajectory not found for " + name);
        }
    }

    private static void create(WindmillTrajectory.WindmillConstraints constraints, TrajectoryState start, double pullback, TrajectoryState end) {
        double elev = (start.elev + end.elev) / 2;
        double arm = (start.arm + end.arm) / 2 - pullback;
        var pos = new WindmillState(
            0,
            new WindmillState.ElevatorState(elev, 0, 0),
            new WindmillState.ArmState(arm, 0, 0)
        );
        create(constraints, start, end, new WindmillState[]{pos});
    }

    private static void create(WindmillTrajectory.WindmillConstraints constraints, TrajectoryState start, TrajectoryState end, WindmillState[] midPositions) {
        WindmillState[] states = new WindmillState[2 + midPositions.length];
        states[0] = start.state;
        states[states.length - 1] = end.state;
        for (int i = 0; i < midPositions.length; i++) {
            states[i + 1] = midPositions[i];
        }
        String name = start.name() + "_TO_" + end.name();
        try {
            trajectories.put(new Pair<>(start, end), new WindmillTrajectory(name, states, constraints));
        } catch (Exception e) {
            System.err.println("Trajectory not found for " + name);
        }
    }

    private static void create(TrajectoryState... trajectoryStates) {
        TrajectoryState start = trajectoryStates[0];
        TrajectoryState end = trajectoryStates[trajectoryStates.length - 1];
        WindmillState[] states = new WindmillState[trajectoryStates.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = trajectoryStates[i].state;
        }
        String name = start.name() + "_TO_" + end.name();
        try {
            WindmillTrajectory trajectory = new WindmillTrajectory(name, states);
            trajectories.put(new Pair<>(start, end), trajectory);
        } catch (Exception e) {
            System.err.println("Trajectory not found for " + name);
        }
    }

    private static void create(WindmillTrajectory.WindmillConstraints constraints, TrajectoryState... trajectoryStates) {
        TrajectoryState start = trajectoryStates[0];
        TrajectoryState end = trajectoryStates[trajectoryStates.length - 1];
        WindmillState[] states = new WindmillState[trajectoryStates.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = trajectoryStates[i].state;
        }
        String name = start.name() + "_TO_" + end.name();
        try {
            WindmillTrajectory trajectory = new WindmillTrajectory(name, states, constraints);
            trajectories.put(new Pair<>(start, end), trajectory);
        } catch (Exception e) {
            System.err.println("Trajectory not found for " + name);
        }
    }

    public static void initialize() {}
}
