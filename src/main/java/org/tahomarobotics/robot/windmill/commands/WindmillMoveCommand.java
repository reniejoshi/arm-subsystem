package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState;
import org.tahomarobotics.robot.windmill.WindmillKinematics;
import org.tahomarobotics.robot.windmill.WindmillState;
import org.tahomarobotics.robot.windmill.WindmillTrajectory;
import org.tinylog.Logger;

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

    // Command

    private WindmillMoveCommand(Pair<TrajectoryState, TrajectoryState> fromTo, WindmillTrajectory trajectory) {
        Logger.info("Created move command from {} to {}.", fromTo.getFirst(), fromTo.getSecond());

        this.fromTo = fromTo;
        this.trajectory = trajectory;

        addRequirements(windmill);
    }

    @Override
    public void initialize() {
        if (!windmill.isAtTargetTrajectoryState()) {
            Logger.error(
                "Windmill was not within tolerance for starting state! Arm was at ({}, {}) but needs to be at ({}, {})", windmill.getWindmillPositionX(),
                windmill.getWindmillPositionY(), fromTo.getFirst().t2d.getX(), fromTo.getFirst().t2d.getY()
            );
            cancel();
            return;
        }

        windmill.setTargetState(fromTo.getSecond());
        windmill.field.getObject("Trajectory").setTrajectory(trajectory.getBackingTrajectory());

        Logger.info("Running trajectory: '{}'", WindmillTrajectory.getFileNameWithExtension(fromTo));
        timer.restart();
    }

    @Override
    public void execute() {
        double time = timer.get();
        try {
            WindmillState state = trajectory.sampleWindmillState(time);
            windmill.setState(state);

            if (DEBUG) {
                Pose2d sample = trajectory.sampleBackingTrajectory(time).poseMeters;
                Logger.info(
                    """
                        Endpoint ({0.0} seconds): ({+0.000;-0.000} meters, {+0.000;-0.000} meters) -> State: ({+0.000;-0.000} meters, {+0.000;-0.000} degrees)
                        """.trim(), time, sample.getX(), sample.getY(), state.elevatorState().heightMeters(),
                    Units.radiansToDegrees(state.armState().angleRadians())
                );
            }
        } catch (WindmillKinematics.KinematicsException e) {
            Logger.error("Kinematics Error: {}", e);
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return windmill.isAtTargetTrajectoryState() || timer.hasElapsed(trajectory.getDuration() + TIME_ELAPSED_TOLERANCE);
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
