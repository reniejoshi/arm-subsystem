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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;

import java.util.List;

import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class SwerveModule {
    private static final Logger logger = LoggerFactory.getLogger(SwerveModule.class);

    // Identifiers

    private final String name;
    private final Translation2d translationOffset;
    @Logged
    private double angularOffset;

    // Hardware

    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder steerEncoder;

    // State

    @Logged
    private SwerveModuleState targetState = new SwerveModuleState();

    // Status Signals

    private final StatusSignal<Angle> steerPosition;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<Angle> driveRotorPosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<AngularAcceleration> driveAcceleration;
    private final StatusSignal<Current> driveCurrent, steerCurrent;

    // Control Requests

    private final VelocityVoltage driveMotorVelocity = new VelocityVoltage(0.0).withEnableFOC(RobotConfiguration.CANIVORE_PHOENIX_PRO);
    private final PositionDutyCycle steerMotorPosition = new PositionDutyCycle(0.0).withEnableFOC(RobotConfiguration.CANIVORE_PHOENIX_PRO);

    private final RobustConfigurator configurator;

    // Initialization

    public SwerveModule(RobotMap.SwerveModuleDescriptor descriptor, double angularOffset) {
        configurator = new RobustConfigurator(logger);

        name = descriptor.moduleName();
        translationOffset = descriptor.offset();
        this.angularOffset = angularOffset;

        driveMotor = new TalonFX(descriptor.driveId(), RobotConfiguration.CANBUS_NAME);
        steerMotor = new TalonFX(descriptor.steerId(), RobotConfiguration.CANBUS_NAME);
        steerEncoder = new CANcoder(descriptor.encoderId(), RobotConfiguration.CANBUS_NAME);

        configurator.configureTalonFX(driveMotor, driveMotorConfiguration, descriptor.moduleName() + " drive motor");
        configurator.configureTalonFX(steerMotor, steerMotorConfiguration, descriptor.encoderId(), descriptor.moduleName() + " steer motor");
        configurator.configureCancoder(steerEncoder, encoderConfiguration, angularOffset, descriptor.moduleName() + " encoder");

        drivePosition = driveMotor.getPosition();
        driveRotorPosition = driveMotor.getRotorPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();

        steerPosition = steerEncoder.getPosition();
        steerVelocity = steerEncoder.getVelocity();

        driveCurrent = driveMotor.getSupplyCurrent();
        steerCurrent = steerMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY,
            drivePosition,
            driveRotorPosition,
            driveVelocity,
            driveAcceleration,
            steerVelocity,
            driveCurrent,
            steerCurrent
        );
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, steerMotor, steerEncoder);
    }

    // Calibration

    public void initializeCalibration() {
        configurator.setCancoderAngularOffset(steerEncoder, 0);
        configurator.setMotorNeutralMode(steerMotor, NeutralModeValue.Coast);
    }

    public double finalizeCalibration() {
        angularOffset = -steerPosition.refresh().getValueAsDouble();
        configurator.setCancoderAngularOffset(steerEncoder, angularOffset);
        configurator.setMotorNeutralMode(steerMotor, NeutralModeValue.Brake);
        return angularOffset;
    }

    public void cancelCalibration() {
        configurator.setCancoderAngularOffset(steerEncoder, angularOffset);
    }

    // Getters

    public Translation2d getTranslationOffset() {
        return translationOffset;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRotations(getSteerAngle()));
    }

    @Logged(name = "steerAngle")
    public double getSteerAngle() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPosition, steerVelocity);
    }

    @Logged(name = "drivePosition")
    public double getDrivePosition() {
        return BaseStatusSignal.getLatencyCompensatedValueAsDouble(drivePosition, driveVelocity) * DRIVE_POSITION_COEFFICIENT;
    }

    @Logged(name = "driveVelocity")
    public double getDriveVelocity() {
        return driveVelocity.getValueAsDouble() * DRIVE_POSITION_COEFFICIENT;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(getSteerAngle()));
    }

    @Logged(name = "driveCurrent")
    public double getDriveCurrent() {
        return driveCurrent.getValueAsDouble();
    }

    @Logged(name = "steerCurrent")
    public double getSteerCurrent() {
        return steerCurrent.getValueAsDouble();
    }

    // Setter

    public void setDesiredState(SwerveModuleState state) {
        double steerAngle = getSteerAngle();

        // Deep-copy the state to prevent mutation bugs
        targetState = new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(state.angle.getDegrees()));

        targetState.optimize(Rotation2d.fromRotations(steerAngle));
        targetState.angle = Rotation2d.fromRotations((targetState.angle.getRotations() % 1.0 + 1.0) % 1.0);
        targetState.speedMetersPerSecond *= targetState.angle.minus(Rotation2d.fromRotations(steerAngle)).getCos();

        driveMotor.setControl(driveMotorVelocity.withVelocity(targetState.speedMetersPerSecond / DRIVE_POSITION_COEFFICIENT));
        steerMotor.setControl(steerMotorPosition.withPosition(targetState.angle.getRotations()));
    }

    // Periodic

    public void periodic() {}

    // Simulation

    private TalonFXSimState driveMotorSimState, steerMotorSimState;
    private CANcoderSimState steerEncoderSimState;
    private DCMotorSim driveMotorSimModel, steerMotorSimModel;

    private double lastUpdateTime;

    public void simulationInit() {
        getStatusSignals().forEach(s -> s.setUpdateFrequency(1000));

        driveMotorSimState = driveMotor.getSimState();
        steerMotorSimState = steerMotor.getSimState();
        steerEncoderSimState = steerEncoder.getSimState();

        driveMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
        steerMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
        steerEncoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;

        driveMotorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                DRIVE_MOTOR_MOI * MOI_SCALING_FACTOR,
                1 / DRIVE_REDUCTION
            ),
            DCMotor.getKrakenX60Foc(1)
        );
        steerMotorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                WHEEL_MOI * MOI_SCALING_FACTOR,
                1 / STEER_REDUCTION
            ),
            DCMotor.getKrakenX60Foc(1)
        );

        lastUpdateTime = Timer.getFPGATimestamp();
    }

    // Rework of Phoenix 6's `SimSwerveDrivetrain` simulation method.
    public void simulationPeriodic(boolean highFidelity) {
        double currentTime = Timer.getFPGATimestamp();
        double dT = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        // Low-fidelity simulation
        if (!highFidelity) {
            driveRotorPosition.refresh();

            driveMotorSimState.setRotorVelocity(driveMotorVelocity.Velocity);
            driveMotorSimState.setRawRotorPosition(driveRotorPosition.getValueAsDouble() + driveMotorVelocity.Velocity * dT);

            steerMotorSimState.setRawRotorPosition(steerMotorPosition.Position);
            steerEncoderSimState.setRawPosition(steerMotorPosition.Position);

            return;
        }

        // Update supply voltage
        double voltage = RobotController.getBatteryVoltage();
        driveMotorSimState.setSupplyVoltage(voltage);
        steerMotorSimState.setSupplyVoltage(voltage);
        steerEncoderSimState.setSupplyVoltage(voltage);
        SmartDashboard.putNumber("Simulation Debugging/Simulation Battery Voltage", voltage);

        // Get motor voltages
        double driveMotorVoltage = driveMotorSimState.getMotorVoltageMeasure().in(Volts);
        double steerMotorVoltage = steerMotorSimState.getMotorVoltageMeasure().in(Volts);
        SmartDashboard.putNumber("Simulation Debugging/Module Simulations/" + name + "/Drive Motor Voltage", driveMotorVoltage);
        SmartDashboard.putNumber("Simulation Debugging/Module Simulations/" + name + "/Steer Motor Voltage", steerMotorVoltage);

        // Calculate position and velocity for given motor voltages
        driveMotorSimModel.setInputVoltage(driveMotorVoltage);
        steerMotorSimModel.setInputVoltage(steerMotorVoltage);

        driveMotorSimModel.update(dT);
        steerMotorSimModel.update(dT);

        // Update rotor position and velocity *pre-gearing*
        driveMotorSimState.setRawRotorPosition(driveMotorSimModel.getAngularPositionRotations() / DRIVE_REDUCTION);
        driveMotorSimState.setRotorVelocity(driveMotorSimModel.getAngularVelocityRPM() / DRIVE_REDUCTION / 60);

        steerMotorSimState.setRawRotorPosition(steerMotorSimModel.getAngularPositionRotations() / STEER_REDUCTION);
        steerMotorSimState.setRotorVelocity(steerMotorSimModel.getAngularVelocityRPM() / STEER_REDUCTION / 60);

        // Sync encoder with steer motor
        steerEncoderSimState.setRawPosition(steerMotorSimModel.getAngularPositionRotations());
        steerEncoderSimState.setVelocity(steerMotorSimModel.getAngularVelocityRPM() / 60);
    }

    // Status Signals

    public List<BaseStatusSignal> getStatusSignals() {
        return List.of(
            drivePosition,
            driveAcceleration,
            driveVelocity,
            steerPosition,
            steerVelocity,
            driveCurrent,
            steerCurrent
        );
    }
}
