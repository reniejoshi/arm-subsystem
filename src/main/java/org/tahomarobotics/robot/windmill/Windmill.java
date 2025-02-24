package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tahomarobotics.robot.windmill.commands.WindmillCommands;
import org.tahomarobotics.robot.windmill.commands.WindmillMoveCommand;
import org.tinylog.Logger;

import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.windmill.WindmillConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Windmill extends SubsystemIF {
    private static final Windmill INSTANCE = new Windmill();

    // -- Member Variables --

    // Hardware

    private final TalonFX elevatorLeftMotor;
    private final TalonFX elevatorRightMotor;
    private final TalonFX armMotor;
    private final CANcoder elevatorEncoder;
    private final CANcoder armEncoder;

    // Status Signals

    private final StatusSignal<Angle> elevatorPosition, armPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity, armVelocity;
    private final StatusSignal<Current> elevatorCurrent, armCurrent;

    @Logged
    private final LoggedStatusSignal.List statusSignals;

    // Control Requests

    private final MotionMagicVoltage elevatorPositionControl = new MotionMagicVoltage(0);
    private final MotionMagicVoltage armPositionControl = new MotionMagicVoltage(0);

    // State

    private double targetHeight;
    private double targetAngle;
    private WindmillState targetState = WindmillState.fromPrevious(0, 0, 0, null);
    private TrajectoryState targetTrajectoryState = TrajectoryState.START;

    @Logged
    private boolean zeroed = false;

    // Trajectory

    public final Field2d field = new Field2d();

    // -- Initialization --

    private Windmill() {
        // Create Hardware

        elevatorLeftMotor = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);
        elevatorRightMotor = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);
        elevatorEncoder = new CANcoder(RobotMap.ELEVATOR_ENCODER);

        armMotor = new TalonFX(RobotMap.ARM_MOTOR);
        armEncoder = new CANcoder(RobotMap.ARM_ENCODER);

        // Configure hardware

        RobustConfigurator.tryConfigureCANcoder("Arm Encoder", armEncoder, armEncoderConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Elevator Left Motor", elevatorLeftMotor, elevatorMotorConfiguration);
        elevatorRightMotor.setControl(new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true));
        RobustConfigurator.tryConfigureTalonFX("Arm Motor", armMotor, armMotorConfiguration);

        // Status Signals

        elevatorPosition = elevatorLeftMotor.getPosition();
        elevatorVelocity = elevatorLeftMotor.getVelocity();
        elevatorCurrent = elevatorLeftMotor.getSupplyCurrent();

        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armCurrent = armMotor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal.List(List.of(
            new LoggedStatusSignal("Elevator Position", elevatorPosition),
            new LoggedStatusSignal("Elevator Velocity", elevatorVelocity),
            new LoggedStatusSignal("Elevator Current", elevatorCurrent),
            new LoggedStatusSignal("Arm Position", armPosition),
            new LoggedStatusSignal("Arm Velocity", armVelocity),
            new LoggedStatusSignal("Arm Current", armCurrent),
            new LoggedStatusSignal("Elevator Encoder Position", elevatorEncoder.getPosition()),
            new LoggedStatusSignal("Elevator Encoder Velocity", elevatorEncoder.getVelocity()),
            new LoggedStatusSignal("Arm Encoder Position", armEncoder.getPosition()),
            new LoggedStatusSignal("Arm Encoder Velocity", armEncoder.getVelocity()),
            new LoggedStatusSignal("Elevator Left Motor Voltage", elevatorLeftMotor.getMotorVoltage()),
            new LoggedStatusSignal("Arm Motor Voltage", armMotor.getMotorVoltage())
        ));

        statusSignals.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);

        ParentDevice.optimizeBusUtilizationForAll(
            elevatorLeftMotor, elevatorRightMotor, elevatorEncoder, armMotor);
    }

    public static Windmill getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        // Publish calibration commands
        SmartDashboard.putData("Calibrate Windmill", WindmillCommands.createCalibrateCommand(this));

        // Calibrate on first enable
        new Trigger(RobotState::isEnabled).onTrue(Commands.runOnce(this::calibrate).onlyIf(() -> !zeroed));
        new Trigger(RobotState::isEnabled).onTrue(Commands.runOnce(this::calibrate).onlyIf(() -> !zeroed));

        // Calibrate with user button
        new Trigger(RobotController::getUserButton).onTrue(WindmillCommands.createUserButtonCalibrateCommand(this));

        return this;
    }

    // -- Calibration --

    public void disableBrakeMode() {
        elevatorLeftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        armMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    }

    public void calibrate() {
        elevatorEncoder.setPosition(0);
        armEncoder.setPosition(ARM_CALIBRATION_POSE / ARM_BELT_REDUCTION);

        zeroed = true;
        Logger.info("Windmill Calibrated");
        enableBrakeMode();
    }

    public void enableBrakeMode() {
        elevatorLeftMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        armMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    // -- Getters --

    @Logged(name = "windmillPosition")
    public Translation2d getWindmillPosition() {
        double armAngleRotations = MathUtil.inputModulus(getArmPosition(), 0, 1);

        return WindmillKinematics.forwardKinematics(
            elevatorPosition.getValueAsDouble(),
            Units.rotationsToRadians(armAngleRotations),
            true
        );
    }

    @Logged(name = "windmillX")
    public double getWindmillPositionX() {
        return getWindmillPosition().getX();
    }

    @Logged(name = "windmillY")
    public double getWindmillPositionY() {
        return getWindmillPosition().getY();
    }

    public WindmillState getCurrentState() {
        WindmillState.ElevatorState elevatorState = new WindmillState.ElevatorState(
            elevatorPosition.getValueAsDouble(),
            elevatorVelocity.getValueAsDouble(),
            elevatorLeftMotor.getAcceleration().refresh().getValueAsDouble()
        );

        WindmillState.ArmState armState = new WindmillState.ArmState(
            armPosition.getValueAsDouble(),
            armVelocity.getValueAsDouble(),
            armMotor.getAcceleration().refresh().getValueAsDouble()
        );

        return new WindmillState(0, elevatorState, armState);
    }

    @Logged(name = "Target State")
    public WindmillState getTargetState() {
        return targetState;
    }

    @Logged(name = "Target Trajectory State")
    public TrajectoryState getTargetTrajectoryState() {
        return targetTrajectoryState;
    }

    public boolean isZeroed() {
        return zeroed;
    }

    // Elevator

    @Logged(name = "elevatorHeight")
    public double getElevatorHeight() {
        return elevatorPosition.getValueAsDouble();
    }

    @Logged(name = "elevatorTarget")
    public double getElevatorTarget() {
        return targetHeight;
    }

    @Logged
    public boolean isElevatorAtPosition() {
        return Math.abs(targetHeight - getElevatorHeight()) <= ELEVATOR_POSITION_TOLERANCE;
    }

    @Logged
    public boolean isElevatorMoving() {
        return Math.abs(elevatorVelocity.refresh().getValueAsDouble()) >= ELEVATOR_VELOCITY_TOLERANCE;
    }

    @Logged
    public boolean isInSecondStage() {
        return getElevatorHeight() > ELEVATOR_LOW_STAGE_MAX;
    }

    // Arm

    @Logged(name = "armPosition")
    public double getArmPosition() {
        return armPosition.getValueAsDouble();
    }

    @Logged(name = "armTarget")
    public double getArmTarget() {
        return targetAngle;
    }

    @Logged
    public boolean isArmAtPosition() {
        return Math.abs(getArmPosition() - targetAngle) < ARM_POSITION_TOLERANCE;
    }

    @Logged
    public boolean isArmMoving() {
        return Math.abs(armVelocity.refresh().getValueAsDouble()) >= ARM_VELOCITY_TOLERANCE;
    }

    @Logged
    public double distanceToTargetTrajectoryState() {
        return targetTrajectoryState.t2d.getDistance(getWindmillPosition());
    }

    @Logged
    public boolean isAtTargetTrajectoryState() {
        return distanceToTargetTrajectoryState() < 0.03;
    }

    // -- Control --

    public void setTargetState(TrajectoryState targetState) {
        this.targetTrajectoryState = targetState;
    }

    public void setState(WindmillState state) {
        targetState = state;

        setElevatorHeight(state.elevatorState().heightMeters());
        setArmPosition(Units.radiansToRotations(state.armState().angleRadians()));
    }

    public void setElevatorHeight(double height) {
        if (!zeroed) {
            Logger.error("Cannot move elevator without calibration!");
            return;
        }

        targetHeight = MathUtil.clamp(height, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        elevatorRightMotor.setControl(
            new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true)
        );
        elevatorLeftMotor.setControl(
            elevatorPositionControl.withPosition(targetHeight)
        );
    }

    public void setArmPosition(double position) {
        if (!zeroed) {
            Logger.error("Cannot move arm without calibration!");
            return;
        }

        targetAngle = position;
        armMotor.setControl(armPositionControl.withPosition(targetAngle));
    }

    public Command createTransitionCommand(TrajectoryState to) {
        return Commands.deferredProxy(() -> {
            Optional<Command> output = WindmillMoveCommand.fromTo(targetTrajectoryState, to);
            if (output.isEmpty()) {
                Logger.error("WindmillMoveCommand from " + targetTrajectoryState + " to " + to + " failed.");
            }
            return output.orElseGet(Commands::none);
        });
    }

    public Command createTransitionToggleCommand(TrajectoryState onTrue, TrajectoryState onFalse) {
        return Commands.deferredProxy(() -> {
            if (targetTrajectoryState == onFalse) {
                Logger.info("Toggling between {} and {}", onFalse, onTrue);
                return createTransitionCommand(onTrue);
            } else if (targetTrajectoryState == onTrue) {
                Logger.info("Toggling between {} and {}", onTrue, onFalse);
                return createTransitionCommand(onFalse);
            } else {
                return Commands.runOnce(() -> Logger.error("Cannot toggle from an untoggleable state."));
            }
        });
    }

    public void stopElevator() {
        elevatorLeftMotor.stopMotor();
    }

    public void stopArm() {
        armMotor.stopMotor();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        statusSignals.refreshAll();
    }

    // -- Overrides --


    @Override
    public void onTeleopInit() {
        (Commands.waitUntil(() -> zeroed))
            .andThen(Commands.runOnce(() -> {
                double angle = MathUtil.inputModulus(getArmPosition(), 0, 1);
                Logger.info("Starting arm angle: {} rotations", angle);
                if (angle > 0.5 && angle < 0.85) {
                    WindmillState collectState;
                    try {
                        collectState = WindmillKinematics.inverseKinematics(0, TrajectoryState.COLLECT.t2d, null, false);
                    } catch (WindmillKinematics.KinematicsException e) {
                        Logger.error("Cannot go to collect! This is a bug: {}", e);
                        return;
                    }
                    setState(collectState);
                    setTargetState(TrajectoryState.COLLECT);
                } else {
                    WindmillState stowState;
                    try {
                        stowState = WindmillKinematics.inverseKinematics(0, TrajectoryState.STOW.t2d, null);
                    } catch (WindmillKinematics.KinematicsException e) {
                        Logger.error("Cannot go to stow! This is a bug: {}", e);
                        return;
                    }
                    setState(stowState);
                    setTargetState(TrajectoryState.STOW);
                }
            })).schedule();
    }

    @Override
    public void onDisabledInit() {
        stopElevator();
        stopArm();
    }

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            // Elevator test
            SysIdTests.characterize(
                "Elevator", this, elevatorLeftMotor,
                Volts.of(0.25).per(Second), Volts.of(1),
                elevatorRightMotor, true
            ),
            // Arm test
            SysIdTests.characterize(
                "Arm", this, armMotor,
                Volts.of(0.25).per(Second), Volts.of(1)
            )
        );
    }
}
