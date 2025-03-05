package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;

import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

public class SwerveModule {
    // Identifiers

    private final String name;
    private final Translation2d translationOffset;
    @AutoLogOutput(key = "Chassis/Modules/{name}/Steer Offset")
    private double angularOffset;

    // Hardware

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    // State

    @AutoLogOutput(key = "Chassis/Modules/{name}/Target State")
    private SwerveModuleState targetState = new SwerveModuleState();

    // Status Signals

    private final StatusSignal<Angle> steerPosition;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<Angle> driveRotorPosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Current> driveCurrent, steerCurrent;

    private final LoggedStatusSignal[] statusSignals;

    // Control Requests

    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0.0).withEnableFOC(
        RobotConfiguration.CANIVORE_PHOENIX_PRO);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0.0).withEnableFOC(
        RobotConfiguration.CANIVORE_PHOENIX_PRO);

    // Initialization

    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        name = descriptor.moduleName();
        translationOffset = descriptor.offset();
        this.angularOffset = angularOffset;

        driveMotor = new TalonFX(descriptor.driveId(), RobotConfiguration.CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), RobotConfiguration.CANBUS_NAME);
        steerEncoder = new CANcoder(descriptor.encoderId(), RobotConfiguration.CANBUS_NAME);

        RobustConfigurator.tryConfigureTalonFX(name + " Drive Motor", driveMotor, createDriveMotorConfiguration());
        RobustConfigurator.tryConfigureTalonFX(
            name + " Steer Motor", steerMotor, createSteerMotorConfiguration(name, descriptor.encoderId()));
        RobustConfigurator.tryConfigureCANcoder(
            name + " Encoder", steerEncoder, createEncoderConfiguration(angularOffset));

        drivePosition = driveMotor.getPosition();
        driveRotorPosition = driveMotor.getRotorPosition();
        driveVelocity = driveMotor.getVelocity();

        steerPosition = steerEncoder.getAbsolutePosition();
        steerVelocity = steerEncoder.getVelocity();

        driveCurrent = driveMotor.getSupplyCurrent();
        steerCurrent = steerMotor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Drive Position", drivePosition),
            new LoggedStatusSignal("Drive Rotor Position", driveRotorPosition),
            new LoggedStatusSignal("Drive Velocity", driveVelocity),
            new LoggedStatusSignal("Steer Position", steerPosition),
            new LoggedStatusSignal("Steer Velocity", steerVelocity),
            new LoggedStatusSignal("Drive Current", driveCurrent),
            new LoggedStatusSignal("Steer Current", steerCurrent),
            new LoggedStatusSignal("Encoder Position", steerEncoder.getPosition()),
            new LoggedStatusSignal("Encoder Velocity", steerEncoder.getVelocity())
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, steerEncoder);
    }

    // Calibration

    public void initializeCalibration() {
        RobustConfigurator.trySetCANcoderAngularOffset(name + " Encoder", steerEncoder, 0);
        RobustConfigurator.trySetMotorNeutralMode(name + " Steer Motor", steerMotor, NeutralModeValue.Coast);
    }

    public double finalizeCalibration() {
        angularOffset = -steerPosition.refresh().getValueAsDouble();
        cancelCalibration();
        return angularOffset;
    }

    public void cancelCalibration() {
        RobustConfigurator.trySetCANcoderAngularOffset(name + " Encoder", steerEncoder, angularOffset);
        RobustConfigurator.trySetMotorNeutralMode(name + " Steer Motor", steerMotor, NeutralModeValue.Brake);
    }

    // Getters

    public String getName() {
        return name;
    }

    public Translation2d getTranslationOffset() {
        return translationOffset;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRotations(getSteerAngle()));
    }

    public double getSteerAngle() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPosition, steerVelocity);
    }

    public double getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            drivePosition, driveVelocity) * DRIVE_POSITION_COEFFICIENT;
    }

    public double getDriveVelocity() {
        return driveVelocity.getValueAsDouble() * DRIVE_POSITION_COEFFICIENT;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(getSteerAngle()));
    }

    public double getDriveCurrent() {
        return driveCurrent.getValueAsDouble();
    }

    public double getSteerCurrent() {
        return steerCurrent.getValueAsDouble();
    }

    // Setter

    public void setDesiredState(SwerveModuleState state) {
        double steerAngle = getSteerAngle();

        // Deep-copy the state to prevent mutation bugs
        targetState = new SwerveModuleState(
            state.speedMetersPerSecond, Rotation2d.fromDegrees(state.angle.getDegrees()));

        targetState.optimize(Rotation2d.fromRotations(steerAngle));
        targetState.angle = Rotation2d.fromRotations((targetState.angle.getRotations() % 1.0 + 1.0) % 1.0);
        targetState.speedMetersPerSecond *= targetState.angle.minus(Rotation2d.fromRotations(steerAngle)).getCos();

        driveMotor.setControl(
            driveMotorVelocity.withVelocity(targetState.speedMetersPerSecond / DRIVE_POSITION_COEFFICIENT));
        steerMotor.setControl(steerMotorPosition.withPosition(targetState.angle.getRotations()));
    }

    // Periodic

    public void periodic() {
        LoggedStatusSignal.log("Chassis/Modules/" + name + "/", statusSignals);
    }

    // Simulation

    private TalonFXSimState driveMotorSimState, steerMotorSimState;
    private CANcoderSimState steerEncoderSimState;

    double lastUpdateTime;

    public void simulationInit() {
        driveMotorSimState = driveMotor.getSimState();
        steerMotorSimState = steerMotor.getSimState();
        steerEncoderSimState = steerEncoder.getSimState();

        driveMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
        steerMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
        steerEncoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        lastUpdateTime = Timer.getTimestamp();
    }

    public void simulationPeriodic() {
        double currentTime = Timer.getTimestamp();
        double dT = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        driveRotorPosition.refresh();

        driveMotorSimState.setRotorVelocity(driveMotorVelocity.Velocity);
        driveMotorSimState.setRawRotorPosition(
            driveRotorPosition.getValueAsDouble() + driveMotorVelocity.Velocity * dT);

        steerMotorSimState.setRawRotorPosition(steerMotorPosition.Position);
        steerEncoderSimState.setRawPosition(steerMotorPosition.Position);

    }

    // Status Signals

    public LoggedStatusSignal[] getStatusSignals() {
        return statusSignals;
    }
}
