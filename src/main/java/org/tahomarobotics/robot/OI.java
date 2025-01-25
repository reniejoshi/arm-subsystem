package org.tahomarobotics.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.commands.TeleopDriveCommand;
import org.tahomarobotics.robot.util.SubsystemIF;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class OI extends SubsystemIF {
    private static final OI INSTANCE = new OI();

    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;

    private static final double DEADBAND = 0.09;

    private final Chassis chassis = Chassis.getInstance();

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController manipController = new CommandXboxController(1);

    // Initialization

    private OI() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
        DriverStation.silenceJoystickConnectionWarning(true);

        configureBindings();
        setDefaultCommands();
    }

    public static OI getInstance() {
        return INSTANCE;
    }

    // Bindings

    public void configureBindings() {
        driveController.a().onTrue(Commands.runOnce(chassis::zeroHeading));
    }

    public void setDefaultCommands() {
        chassis.setDefaultCommand(new TeleopDriveCommand(
            this::getDriveLeftY, this::getDriveLeftX, this::getDriveRightX
        ));
    }

    // Inputs

    @Logged(name = "Controllers/Drive/LeftX")
    public double getDriveLeftX() {
        return -desensitizePowerBased(driveController.getLeftX(), TRANSLATIONAL_SENSITIVITY);
    }

    @Logged(name = "Controllers/Drive/LeftY")
    public double getDriveLeftY() {
        return -desensitizePowerBased(driveController.getLeftY(), TRANSLATIONAL_SENSITIVITY);
    }

    @Logged(name = "Controllers/Drive/RightX")
    public double getDriveRightX() {
        return -desensitizePowerBased(driveController.getRightX(), ROTATIONAL_SENSITIVITY);
    }

    // Helper Methods

    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}