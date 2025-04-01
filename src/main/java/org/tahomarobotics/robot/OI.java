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

package org.tahomarobotics.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.auto.AutonomousConstants;
import org.tahomarobotics.robot.auto.commands.DriveToPoseV5Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisCommands;
import org.tahomarobotics.robot.climber.Climber;
import org.tahomarobotics.robot.climber.commands.ClimberCommands;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorCommands;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.grabber.GrabberCommands;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.indexer.IndexerCommands;
import org.tahomarobotics.robot.lights.LED;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.game.GamePiece;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tahomarobotics.robot.vision.Vision;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;
import org.tahomarobotics.robot.windmill.commands.WindmillCommands;
import org.tinylog.Logger;

import java.util.List;
import java.util.Set;
import java.util.function.Function;

public class OI extends SubsystemIF {
    private static final OI INSTANCE = new OI();

    // -- Constants --

    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;

    private static final double DEADBAND = 0.09;
    private static final double TRIGGER_DEADBAND = 0.05;

    private static final double DOUBLE_PRESS_TIMEOUT = 0.35;
    private static final double ARM_UP_DISTANCE = AutonomousConstants.APPROACH_DISTANCE_BLEND_FACTOR + Units.inchesToMeters(6);

    // -- Subsystems --

    private final Indexer indexer = Indexer.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Chassis chassis = Chassis.getInstance();
    private final Windmill windmill = Windmill.getInstance();
    private final Grabber grabber = Grabber.getInstance();
    private final LED led = LED.getInstance();

    private final List<SubsystemIF> subsystems;

    // -- Controllers --

    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController lessImportantController = new CommandXboxController(1);

    // -- Initialization --

    private OI() {

        subsystems = RobotConfiguration.isClimberEnabled() ?
            List.of(indexer, collector, chassis, windmill, grabber, led, Climber.getInstance()) :
            List.of(indexer, collector, chassis, windmill, grabber, led);

        CommandScheduler.getInstance().unregisterSubsystem(this);
        DriverStation.silenceJoystickConnectionWarning(true);

        configureControllerBindings();
        configureLessImportantControllerBindings();

        setDefaultCommands();
    }

    public static OI getInstance() {
        return INSTANCE;
    }

    // -- Bindings --

    public void configureControllerBindings() {
        // -- D-Pad --

        // Up - Drives to nearest coral
        controller.povUp().whileTrue(
            Commands.deferredProxy(
                () -> {
                    if (!RobotConfiguration.FEATURE_CORAL_DETECTION) { return Commands.runOnce(() -> Logger.warn("Coral detection is disabled!")); }

                    return new DriveToPoseV5Command(
                        -1, Vision.getInstance()::getCoralPosition, chassis.getPose().getTranslation()
                    );
                }
            )
        );

        // Down - Orients to zero heading
        controller.povDown().onTrue(Commands.runOnce(chassis::orientToZeroHeading));

        // Left - Eject the collector and indexer
        Pair<Command, Command> ejectCommands = CollectorCommands.createEjectCommands(collector);
        controller.povLeft().onTrue(ejectCommands.getFirst()).onFalse(ejectCommands.getSecond());

        Pair<Command, Command> indexerEjectingCommands = IndexerCommands.createIndexerEjectingCommands(indexer);
        controller.povLeft().onTrue(indexerEjectingCommands.getFirst()).onFalse(indexerEjectingCommands.getSecond());

        // -- Bumpers --

        // Left - Toggle collector deployment
        controller.leftBumper().onTrue(CollectorCommands.createDeploymentControlCommand(collector));

        // Right - Auto-align to the closest pole
        controller.rightBumper().whileTrue(
            Commands.deferredProxy(
                        () -> {
                            int nearestIndex = AutonomousConstants.getNearestReefPoleIndex(Chassis.getInstance().getPose().getTranslation());
                            // Drive to Pose
                            AutonomousConstants.Objective pole =
                                AutonomousConstants.getObjectiveForPole(nearestIndex, AutonomousConstants.getAlliance())
                                    .fudgeY(0);
                            var dtp = pole.driveToPoseV4Command();

                            boolean isHighAlgae = (nearestIndex / 2) % 2 == 0;

                            Command scoreToDescore = Commands.none();
                            if ((windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.L4
                                 || windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.L3
                                 || windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.L2)) {
                                if (isHighAlgae) {
                                    scoreToDescore = WindmillCommands.createScoreToHighAlgaeDescoreCommand(windmill);
                                } else {
                                    scoreToDescore = WindmillCommands.createScoreToLowAlgaeDescoreCommand(windmill);
                                }
                            }

                            return Commands.parallel(
                                dtp.andThen(Commands.waitSeconds(0.75)).finallyDo(grabber::transitionToDisabled),
                                dtp.runWhen(
                                    () -> dtp.getDistanceToWaypoint() < AutonomousConstants.AUTO_SCORE_DISTANCE,
                                    grabber.runOnce(grabber::transitionToScoring)
                                )
                            ).andThen(scoreToDescore.onlyIf(() -> (controller.y().getAsBoolean() || controller.b().getAsBoolean() || controller.a().getAsBoolean())
                                                                  && (windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.L4
                                                                      || windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.L3
                                                                      || windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.L2)));
                        }
                    )
                    .onlyIf(() -> collector.getCollectionMode() == GamePiece.CORAL)
        );

        controller.rightBumper().onTrue(Commands.deferredProxy(
            () -> (collector.getCollectionMode() == GamePiece.ALGAE) ? WindmillCommands.createAlgaeThrowCommmand(windmill) : Commands.none()));

        // -- Joystick Press --

        // Left
        controller.leftStick().onTrue(collector.runOnce(collector::toggleCollectionMode).andThen(windmill.createSyncCollectionModeCommand()));

        // Right - Stow <-> Collect / Go to previous state if out of tolerance
        controller.rightStick().onTrue(
            Commands.defer(
                () -> {
                    if (windmill.isAtTargetTrajectoryState()) {
                        if (collector.getCollectionMode().equals(GamePiece.CORAL)) {
                            return windmill.createTransitionToggleCommand(
                                WindmillConstants.TrajectoryState.CORAL_COLLECT, WindmillConstants.TrajectoryState.STOW);
                        } else {
                            if (windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.STOW) {
                                return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.ALGAE_COLLECT);
                            } else {
                                return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.STOW);
                            }
                        }
                    } else {
                        return windmill.createResetToPreviousState(true);
                    }
                }, Set.of(windmill)
            )
        );

        // -- Triggers --

        // Left

        //// Collecting
        Pair<Command, Command> collectorCommands = CollectorCommands.createCollectorControlCommands(collector);
        controller.leftTrigger().onTrue(collectorCommands.getFirst()).onFalse(collectorCommands.getSecond());

        Pair<Command, Command> indexerCommands = IndexerCommands.createIndexerCommands(indexer);
        controller.leftTrigger().onTrue(indexerCommands.getFirst()).onFalse(indexerCommands.getSecond());

        Pair<Command, Command> grabberCommands = GrabberCommands.createGrabberCommands(grabber);
        controller.leftTrigger(TRIGGER_DEADBAND).onTrue(grabberCommands.getFirst()).onFalse(grabberCommands.getSecond());

        //// Climber Rollers
        if (RobotConfiguration.isClimberEnabled()) {
            Climber climber = Climber.getInstance();
            controller.leftTrigger().onTrue(climber.runOnce(climber::runRollers).onlyIf(() -> climber.getClimbState() == Climber.ClimberState.DEPLOYED))
                      .onFalse(climber.runOnce(climber::disableRollers));
        }

        // Right - Eject everything
        Pair<Command, Command> scoreCommands = CollectorCommands.createEjectCommands(collector);
        controller.rightTrigger().onTrue(scoreCommands.getFirst()).onFalse(scoreCommands.getSecond());

        Pair<Command, Command> indexerScoringCommands = IndexerCommands.createIndexerEjectingCommands(indexer);
        controller.rightTrigger().onTrue(indexerScoringCommands.getFirst())
                  .onFalse(indexerScoringCommands.getSecond());

        Pair<Command, Command> grabberScoringCommands = GrabberCommands.createGrabberScoringCommands(grabber);
        controller.rightTrigger(TRIGGER_DEADBAND).onTrue(grabberScoringCommands.getFirst())
                  .onFalse(grabberScoringCommands.getSecond());

        // -- Start & Back --

        // Start
        if (RobotConfiguration.isClimberEnabled()) {
            controller.start().onTrue(ClimberCommands.getClimberCommand());
        }
        // -- ABXY --

        // A - L2 / Low Algae De-score
        controller.a().onTrue(Commands.defer(
            () -> windmill.createTransitionCommand(
                collector.getCollectionMode() == GamePiece.CORAL ?
                    WindmillConstants.TrajectoryState.L2 :
                    WindmillConstants.TrajectoryState.LOW_DESCORE
            ), Set.of(windmill)
        ));

        // B - L3 / High Algae De-score
        controller.b().onTrue(Commands.defer(
            () -> windmill.createTransitionCommand(
                collector.getCollectionMode() == GamePiece.CORAL ?
                    WindmillConstants.TrajectoryState.L3 :
                    WindmillConstants.TrajectoryState.HIGH_DESCORE
            ), Set.of(windmill)
        ));

        // X - L1 / Algae Processor
        controller.x().onTrue(Commands.defer(
            () -> collector.getCollectionMode() == GamePiece.CORAL ?
                windmill.createTransitionCommand(WindmillConstants.TrajectoryState.L1) :
                Commands.runOnce(() -> collector.deploymentForceStateTransition(CollectorConstants.TargetDeploymentState.CORAL_COLLECT))
                    .andThen(WindmillCommands.createToggleProcessorCommand(windmill)),
            Set.of(windmill)
        ));

        // Y - L4
        controller.y().onTrue(Commands.deferredProxy(
            () -> {
                if (collector.getCollectionMode().equals(GamePiece.CORAL)) {
                    return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.L4);
                } else {
                    if (windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.ALGAE_PRESCORE) {
                        return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.STOW);
                    }
                    return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.ALGAE_PRESCORE);
                }
            }
        ));
    }

    public void configureLessImportantControllerBindings() {
        // -- D-Pad --

        // Left - Shift Auto-Align Left
        lessImportantController
            .povLeft()
            .onTrue(Commands.runOnce(() -> chassis.incrementAutoAligningOffset(AutonomousConstants.FUDGE_INCREMENT)).ignoringDisable(true));

        // Right - Shift Auto-Align Right
        lessImportantController
            .povRight()
            .onTrue(Commands.runOnce(() -> chassis.incrementAutoAligningOffset(-AutonomousConstants.FUDGE_INCREMENT)).ignoringDisable(true));

        // -- ABXY --

        // X - RESET
        lessImportantController.x().onTrue(
            Commands.runOnce(
                        () -> {
                            windmill.setTargetState(WindmillConstants.TrajectoryState.STOW);
                            windmill.setArmPosition(0.25);
                        }
                    )
                    .andThen(Commands.waitUntil(windmill::isArmAtPosition))
                    .andThen(Commands.runOnce(() -> windmill.setElevatorHeight(WindmillConstants.ELEVATOR_MIN_POSE)))
        );
    }

    // -- Helper Methods --

    @SuppressWarnings("SuspiciousNameCombination")
    public void setDefaultCommands() {
        chassis.setDefaultCommand(ChassisCommands.createTeleOpDriveCommand(
            chassis,
            this::getLeftY, this::getLeftX, this::getRightX
        ));

        grabber.setVelocitySupplier(() -> MathUtil.applyDeadband(controller.getRightTriggerAxis(), TRIGGER_DEADBAND) - MathUtil.applyDeadband(controller.getLeftTriggerAxis(), TRIGGER_DEADBAND));
    }

    // -- Inputs --

    public double getLeftX() {
        return -desensitizePowerBased(controller.getLeftX(), TRANSLATIONAL_SENSITIVITY);
    }

    public double getLeftY() {
        return -desensitizePowerBased(controller.getLeftY(), TRANSLATIONAL_SENSITIVITY);
    }

    public double getRightX() {
        return -desensitizePowerBased(controller.getRightX(), ROTATIONAL_SENSITIVITY);
    }

    // -- SysID --

    public void initializeSysId() {
        // Clear all bound triggers and default commands

        CommandScheduler scheduler = CommandScheduler.getInstance();

        subsystems.forEach(scheduler::removeDefaultCommand);
        scheduler.getActiveButtonLoop().clear();
        scheduler.cancelAll();

        // Allow for selection of tests

        SendableChooser<SysIdTests.Test> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Nothing", null);

        subsystems.stream().flatMap(s -> s.getSysIdTests().stream())
                  .forEachOrdered(test -> chooser.addOption(test.name(), test));

        SmartDashboard.putData("SysId Test", chooser);

        // Create proxy commands for controller bindings

        Function<Function<SysIdTests.Test, Command>, Command> getCommand =
            (command) -> Commands.deferredProxy(() -> {
                SysIdTests.Test selected = chooser.getSelected();
                if (selected == null) {
                    return Commands.none();
                }
                return command.apply(selected);
            });

        Command quasistaticForward = getCommand.apply(SysIdTests.Test::quasistaticForward);
        Command quasistaticReverse = getCommand.apply(SysIdTests.Test::quasistaticReverse);
        Command dynamicForward = getCommand.apply(SysIdTests.Test::dynamicForward);
        Command dynamicReverse = getCommand.apply(SysIdTests.Test::dynamicReverse);

        // Register commands to the controller

        controller.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        controller.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        controller.povUp().whileTrue(quasistaticForward);
        controller.povDown().whileTrue(quasistaticReverse);
        controller.povLeft().whileTrue(dynamicForward);
        controller.povRight().whileTrue(dynamicReverse);
    }

    public void cleanUpSysId() {
        // Clear all bound triggers
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Rebind proper controls
        configureControllerBindings();
        setDefaultCommands();
    }

    // -- Helper Methods --

    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}
