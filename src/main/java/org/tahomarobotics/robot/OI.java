package org.tahomarobotics.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.auto.AutonomousConstants;
import org.tahomarobotics.robot.auto.DriveToPoseV3Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisCommands;
import org.tahomarobotics.robot.climber.Climber;
import org.tahomarobotics.robot.climber.commands.ClimberCommands;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorCommands;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.grabber.GrabberCommands;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.indexer.IndexerCommands;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.game.GamePiece;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;

import java.util.List;
import java.util.function.Function;

public class OI extends SubsystemIF {
    private static final OI INSTANCE = new OI();

    // -- Constants --

    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;

    private static final double DEADBAND = 0.09;

    // -- Subsystems --

    private final Indexer indexer = Indexer.getInstance();
    private final Climber climber = Climber.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Chassis chassis = Chassis.getInstance();
    private final Windmill windmill = Windmill.getInstance();
    private final Grabber grabber = Grabber.getInstance();

    private final List<SubsystemIF> subsystems = List.of(indexer, collector, chassis, climber, windmill, grabber);

    // -- Controllers --

    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController lessImportantController = new CommandXboxController(1);

    // -- Initialization --

    private OI() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
        DriverStation.silenceJoystickConnectionWarning(true);

        configureBindings();
        setDefaultCommands();
    }

    public static OI getInstance() {
        return INSTANCE;
    }

    // -- Bindings --

    public void configureBindings() {
        // Chassis

        controller.povDown().onTrue(Commands.runOnce(chassis::orientToZeroHeading));

        controller.rightBumper().whileTrue(Commands.deferredProxy(() -> new DriveToPoseV3Command(
            AutonomousConstants.getNearestReefPoleScorePosition(Chassis.getInstance().getPose().getTranslation())
        )));

        // Collector

        controller.leftBumper().onTrue(CollectorCommands.createDeploymentControlCommand(collector));
        controller.leftStick().onTrue(collector.runOnce(collector::toggleCollectionMode).andThen(windmill.createSyncCollectionModeCommand()));

        Pair<Command, Command> ejectCommands = CollectorCommands.createEjectCommands(collector);
        controller.povLeft().onTrue(ejectCommands.getFirst()).onFalse(ejectCommands.getSecond());

        Pair<Command, Command> scoreCommands = CollectorCommands.createEjectCommands(collector);
        controller.rightTrigger().onTrue(scoreCommands.getFirst()).onFalse(scoreCommands.getSecond());

        Pair<Command, Command> collectorCommands = CollectorCommands.createCollectorControlCommands(collector);
        controller.leftTrigger().onTrue(collectorCommands.getFirst()).onFalse(collectorCommands.getSecond());

        // Indexer

        Pair<Command, Command> indexerCommands = IndexerCommands.createIndexerCommands(indexer);
        controller.leftTrigger().onTrue(indexerCommands.getFirst()).onFalse(indexerCommands.getSecond());

        Pair<Command, Command> indexerEjectingCommands = IndexerCommands.createIndexerEjectingCommands(indexer);
        controller.povLeft().onTrue(indexerEjectingCommands.getFirst())
                  .onFalse(indexerEjectingCommands.getSecond());

        Pair<Command, Command> indexerScoringCommands = IndexerCommands.createIndexerEjectingCommands(indexer);
        controller.rightTrigger().onTrue(indexerScoringCommands.getFirst())
                  .onFalse(indexerScoringCommands.getSecond());

        //Grabber

        Pair<Command, Command> grabberCommands = GrabberCommands.createGrabberCommands(grabber);
        controller.leftTrigger().onTrue(grabberCommands.getFirst()).onFalse(grabberCommands.getSecond());

        Pair<Command, Command> grabberScoringCommands = GrabberCommands.createGrabberScoringCommands(grabber);
        controller.rightTrigger().onTrue(grabberScoringCommands.getFirst())
                  .onFalse(grabberScoringCommands.getSecond());


        // Elevator
        // TODO: Temporary Controls

        SmartDashboard.putData(
            "Elevator Up", Commands.runOnce(
                () -> windmill.setElevatorHeight(WindmillConstants.ELEVATOR_HIGH_POSE))
        );

        SmartDashboard.putData(
            "Elevator Down", Commands.runOnce(
                () -> windmill.setElevatorHeight(WindmillConstants.ELEVATOR_LOW_POSE))
        );

        // Arm
        // TODO: Temporary Controls

        controller.start().onTrue(ClimberCommands.getClimberCommand());

        SmartDashboard.putData(
            "Arm Upright", Commands.runOnce(
                () -> windmill.setArmPosition(WindmillConstants.ARM_UPRIGHT_POSE))
        );

        SmartDashboard.putData(
            "Arm Horizontal", Commands.runOnce(
                () -> windmill.setArmPosition(WindmillConstants.ARM_TEST_POSE))
        );

        controller.y().onTrue(windmill.createTransitionCommand(WindmillConstants.TrajectoryState.L4));
        controller.b().onTrue(Commands.deferredProxy(() -> {
            if (collector.getCollectionMode() == GamePiece.CORAL) {
                return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.L3);
            } else {
                return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.HIGH_DESCORE);
            }
        }));
        controller.a().onTrue(Commands.deferredProxy(() -> {
            if (collector.getCollectionMode() == GamePiece.CORAL) {
                return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.L2);
            } else {
                return windmill.createTransitionCommand(WindmillConstants.TrajectoryState.LOW_DESCORE);
            }
        }));
        controller.x().onTrue(Commands.deferredProxy(() -> {
            if (windmill.isAtTargetTrajectoryState()) {
                return windmill.createTransitionToggleCommand(WindmillConstants.TrajectoryState.COLLECT, WindmillConstants.TrajectoryState.STOW);
            } else { return windmill.createResetToClosestCommand(); }
        }));

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

        lessImportantController.povLeft()
                               .onTrue(Commands.runOnce(() -> chassis.incrementAutoAligningOffset(AutonomousConstants.FUDGE_INCREMENT)).ignoringDisable(true));
        lessImportantController.povRight()
                               .onTrue(Commands.runOnce(() -> chassis.incrementAutoAligningOffset(-AutonomousConstants.FUDGE_INCREMENT)).ignoringDisable(true));

        SmartDashboard.putData(
            "Set Elevator Collecting", Commands.runOnce(
                () -> windmill.setElevatorHeight(WindmillConstants.ELEVATOR_COLLECT_POSE))
        );
        SmartDashboard.putData(
            "Set Arm Collecting", Commands.runOnce(() -> windmill.setArmPosition(WindmillConstants.ARM_COLLECT_POSE)));
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public void setDefaultCommands() {
        chassis.setDefaultCommand(ChassisCommands.createTeleOpDriveCommand(
            chassis,
            this::getLeftY, this::getLeftX, this::getRightX
        ));
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
        configureBindings();
        setDefaultCommands();
    }

    // -- Helper Methods --

    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}