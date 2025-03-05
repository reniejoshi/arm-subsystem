package org.tahomarobotics.robot.check;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.climber.Climber;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;
import org.tinylog.Logger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;

public class SystemCheck {
    private static Command ensureNominal(Measurement... measurements) {
        Map<Measurement, List<Double>> values = new HashMap<>(measurements.length);
        for (Measurement m : measurements) { values.put(m, new ArrayList<>()); }

        return new FunctionalCommand(
            () -> values.values().forEach(List::clear),
            () -> values.forEach((m, v) -> v.add(m.measure.get())),
            interrupted -> values.forEach((m, v) -> {
                v.removeIf(n -> n < 0);
                Supplier<DoubleStream> currents = () -> v.stream().mapToDouble(d -> d);

                double mean = currents.get().average().orElse(0);
                double max = currents.get().max().orElse(0);

                if (m.min > mean || m.max < mean) {
                    Logger.error(
                        "{} is not nominal!\nMean current was {}A, which is outside of the nominal range of [{}A, {}A].",
                        m.name, mean, m.min, m.max
                    );
                    SmartDashboard.putBoolean("Checks/Results/" + m.name + "/Passed", false);
                } else {
                    SmartDashboard.putBoolean("Checks/Results/" + m.name + "/Passed", true);
                }

                SmartDashboard.putNumber("Checks/Results/" + m.name + "/Mean", mean);
                SmartDashboard.putNumber("Checks/Results/" + m.name + "/Max", max);
            }),
            () -> false
        );
    }

    // Collector -> Deploy -> Run Collection -> Stow
    public static Command createCollectorTestCommand(Collector collector) {
        return Commands.race(
            ensureNominal(
                new Measurement("Collector", collector::getCollectorCurrent, 5, 7),
                new Measurement("Left Deploy", collector::getLeftDeploymentCurrent, 0.5, 1.25),
                new Measurement("Right Deploy", collector::getRightDeploymentCurrent, 0.5, 1.25)
            ),
            collector.runOnce(collector::deploymentTransitionToCollect)
                     .andThen(Commands.waitUntil(collector::isAtTargetDeploymentState))
                     .andThen(collector.runOnce(collector::collectorTransitionToCollecting))
                     .andThen(Commands.waitSeconds(3))
                     .andThen(collector.runOnce(collector::deploymentTransitionToStow))
                     .andThen(Commands.waitUntil(collector::isAtTargetDeploymentState))
        );
    }

    // Indexer -> Run
    public static Command createIndexerTestCommand(Indexer indexer) {
        return Commands.race(
            ensureNominal(new Measurement("Indexer", indexer::getCurrent, 1.5, 2.25)),
            indexer.runOnce(indexer::transitionToCollecting).andThen(Commands.waitSeconds(3)).andThen(indexer.runOnce(indexer::transitionToDisabled))
        );
    }

    // Run Grabber
    public static Command createGrabberTestCommand(Grabber grabber) {
        return Commands.race(
            ensureNominal(new Measurement("Grabber", grabber::getCurrent, 5.5, 7.5)),
            grabber.runOnce(grabber::transitionToCollecting).andThen(Commands.waitSeconds(3)).andThen(grabber.runOnce(grabber::transitionToDisabled))
        );
    }

    // Run Climber
    public static Command createClimberTestCommand(Climber climber) {
        return Commands.race(
            ensureNominal(
                new Measurement("Climber Lead", climber::getLeadCurrent, .025, .15),
                new Measurement("Climber Follow", climber::getFollowerCurrent, .025, .15)
            ),
            climber.runOnce(climber::deploy)
                   .andThen(Commands.waitUntil(climber::isAtTargetPosition))
                   .andThen(climber.runOnce(climber::stow))
                   .andThen(Commands.waitUntil(climber::isAtTargetPosition))
        );
    }

    // Elevator -> Stow to L3 -> Arm from 0 to PI -> Stow
    public static Command createWindmillTestCommand(Windmill windmill) {
        return Commands.race(
            ensureNominal(
                new Measurement("Elevator Left", windmill::getElevatorLeftCurrent, 1, 3),
                new Measurement("Elevator Right", windmill::getElevatorRightCurrent, 1, 3),
                new Measurement("Arm", windmill::getArmCurrent, 1, 3)
            ),
            windmill.runOnce(() -> windmill.setElevatorHeight(WindmillConstants.ELEVATOR_MID_POSE))
                    .andThen(Commands.waitUntil(windmill::isElevatorAtPosition))
                    .andThen(windmill.runOnce(() -> windmill.setArmPosition(0.05)))
                    .andThen(Commands.waitUntil(windmill::isArmAtPosition))
                    .andThen(windmill.runOnce(() -> windmill.setArmPosition(0.45)))
                    .andThen(Commands.waitUntil(windmill::isArmAtPosition))
                    .andThen(windmill.runOnce(() -> windmill.setArmPosition(0.25)))
                    .andThen(Commands.waitUntil(windmill::isArmAtPosition))
                    .andThen(windmill.runOnce(() -> windmill.setElevatorHeight(WindmillConstants.ELEVATOR_MIN_POSE)))
                    .andThen(Commands.waitUntil(windmill::isElevatorAtPosition))
        );
    }

    // Chassis Separate
    public static Command createChassisTestCommand(Chassis chassis) {
        ChassisSpeeds speeds = new ChassisSpeeds();
        Timer timer = new Timer();

        return Commands.race(
            ensureNominal(
                chassis.getModules().stream().flatMap(
                    m -> Stream.of(
                        new Measurement(m.getName() + " Drive", m::getDriveCurrent, 1, 3),
                        new Measurement(m.getName() + " Steer", m::getSteerCurrent, 0.25, 0.75)
                    )
                ).toArray(Measurement[]::new)
            ),
            chassis.startRun(
                timer::restart, () -> {
                    double t = Math.PI * timer.get();
                    speeds.vxMetersPerSecond = Math.cos(t) * ChassisConstants.MAX_VELOCITY;
                    speeds.vyMetersPerSecond = Math.sin(t) * ChassisConstants.MAX_VELOCITY;
                    chassis.drive(speeds, false);
                }
            ).withTimeout(3).andThen(() -> {
                timer.stop();
                chassis.drive(new ChassisSpeeds());
            })
        );
    }

    public static void initialize() {
        SmartDashboard.putData("Checks/Run Collector", createCollectorTestCommand(Collector.getInstance()));
        SmartDashboard.putData("Checks/Run Indexer", createIndexerTestCommand(Indexer.getInstance()));
        SmartDashboard.putData("Checks/Run Grabber", createGrabberTestCommand(Grabber.getInstance()));
        SmartDashboard.putData("Checks/Run Climber", createClimberTestCommand(Climber.getInstance()));
        SmartDashboard.putData("Checks/Run Windmill", createWindmillTestCommand(Windmill.getInstance()));
        SmartDashboard.putData("Checks/Run Chassis", createChassisTestCommand(Chassis.getInstance()));
    }

    private record Measurement(String name, Supplier<Double> measure, double min, double max) {}
}
