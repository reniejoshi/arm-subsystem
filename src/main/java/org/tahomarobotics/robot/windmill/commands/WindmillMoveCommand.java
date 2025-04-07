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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.util.motion.MotionProfile;
import org.tahomarobotics.robot.util.motion.MotionState;
import org.tahomarobotics.robot.windmill.Dynamics;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState;
import org.tahomarobotics.robot.windmill.WindmillState;
import org.tahomarobotics.robot.windmill.WindmillTrajectory;
import org.tinylog.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class WindmillMoveCommand extends Command {
    private static final boolean DEBUG = false;
    private static final double TIME_ELAPSED_TOLERANCE = 0.05;

    // Subsystems

    private final Windmill windmill = Windmill.getInstance();

    // Trajectory

    private final Pair<TrajectoryState, TrajectoryState> fromTo;
    private final WindmillTrajectory trajectory;

    // State

    private final Timer timer = new Timer();

    boolean broken = false;

    private final Dynamics dynamics = new Dynamics();

    private boolean isProfileDone = false;
    // Command

    private WindmillMoveCommand(Pair<TrajectoryState, TrajectoryState> fromTo, WindmillTrajectory trajectory) {
        this.fromTo = fromTo;
        this.trajectory = trajectory;

        addRequirements(windmill);
    }

    @Override
    public void initialize() {
        // TODO: why are we checking for at target state
        if (!windmill.getTargetTrajectoryState().equals(fromTo.getFirst())) {
            Logger.error(
                "Windmill's target state {} does not equal the starting state {}!", windmill.getTargetTrajectoryState(), fromTo.getFirst()
            );
            broken = true;
            return;
        }
        windmill.setIsRunningTrajectory(true);

        windmill.setTargetState(fromTo.getSecond());

        isProfileDone = false;

        if(fromTo.getSecond() != TrajectoryState.STOW) windmill.setWillMoveToL4OnAutoAlign(false);

        Logger.info("Running trajectory: '{}' ({} seconds)", trajectory.name, trajectory.getTotalTimeSeconds());
        timer.restart();
    }

    MotionState elevState = new MotionState();
    MotionState armState = new MotionState();

    @Override
    public void execute() {
        if (broken) { return; }

        double time = timer.get();
        isProfileDone = trajectory.sample(time, elevState, armState);

        WindmillState state = new WindmillState(
            time,
            new WindmillState.ElevatorState(elevState.position, elevState.velocity, elevState.acceleration),
            new WindmillState.ArmState(armState.position, armState.velocity, armState.acceleration)
        );

        Dynamics.Voltages ffVoltages = dynamics.inverseDynamics(state, 0.0, false);

        windmill.setStateWithFeedForword(state, ffVoltages.elevatorVoltage(), ffVoltages.armVoltage());

                                         var current = windmill.getCurrentState();
        Dynamics.Voltages currentVoltages = windmill.getVoltages();

        if (DEBUG) {
            Logger.info(
                """
                    Endpoint ({0.0} seconds): State: ({+0.000;-0.000} meters, {+0.000;-0.000} degrees)
                    """.trim(), time, state.elevatorState().heightMeters(),
                Units.radiansToDegrees(state.armState().angleRadians())
            );
        }
    }

    @Override
    public boolean isFinished() {
        return broken || isProfileDone && (windmill.isAtTargetTrajectoryState() || timer.hasElapsed(trajectory.getTotalTimeSeconds() + TIME_ELAPSED_TOLERANCE));
    }

    @Override
    public void end(boolean interrupted) {
        windmill.setState(fromTo.getSecond().state);
        windmill.setIsRunningTrajectory(false);
        Logger.info("Ran trajectory: '{}'", trajectory.name);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public String getName() {
        return "Windmill Move Command";
    }

    // -- Helpers --

    public static Optional<Command> fromTo(TrajectoryState from, TrajectoryState to) {
        return WindmillTrajectories.getTrajectory(from, to).map(t -> new WindmillMoveCommand(Pair.of(from, to), t));
    }
}
