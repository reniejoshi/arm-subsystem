package org.tahomarobotics.robot.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.climber.commands.ClimberCommands;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tinylog.Logger;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.climber.ClimberConstants.climberMotorConfig;

public class Climber extends SubsystemIF {
    private final static Climber INSTANCE = new Climber();

    // -- Member Variables --

    // Hardware

    private final TalonFX climberMotor = new TalonFX(RobotMap.CLIMBER_MOTOR);
    private final TalonFX climberFollower = new TalonFX(RobotMap.CLIMBER_FOLLOWER);
    private final VictorSPX ratchetSolenoid = new VictorSPX(RobotMap.CLIMBER_RATCHET_SOLENOID);

    // Status Signals

    private final LoggedStatusSignal[] statusSignals;

    private final StatusSignal<Current> climberMotorCurrent;
    private final StatusSignal<Current> climberFollowerCurrent;
    private final StatusSignal<Angle> climberMotorPosition;

    // Control Requests

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    // State

    @AutoLogOutput(key = "Climber/State")
    private ClimberState climbState = ClimberState.ZEROED;
    @AutoLogOutput(key = "Climber/Target Position")
    private double targetPosition;
    @AutoLogOutput(key = "Climber/Solenoid Voltage")
    private double solenoidVoltage;

    // -- Initialization --

    private Climber() {
        RobustConfigurator.tryConfigureTalonFX("Climber Motor", climberMotor, climberMotorConfig);
        climberFollower.setControl(new Follower(RobotMap.CLIMBER_MOTOR, true));

        climberMotorCurrent = climberMotor.getSupplyCurrent();
        climberFollowerCurrent = climberFollower.getSupplyCurrent();
        climberMotorPosition = climberMotor.getPosition();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Climber Motor Position", climberMotorPosition),
            new LoggedStatusSignal("Climber Follower Position", climberFollower.getPosition()),
            new LoggedStatusSignal("Climber Motor Current", climberMotorCurrent),
            new LoggedStatusSignal("Climber Follower Current", climberFollowerCurrent),
            new LoggedStatusSignal("Climber Motor Voltage", climberMotor.getMotorVoltage()),
            new LoggedStatusSignal("Climber Follower Voltage", climberFollower.getMotorVoltage())
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(climberMotor, climberFollower);
    }

    public static Climber getInstance() {
        return INSTANCE;
    }

    @Override
    public Climber initialize() {
        new Trigger(() -> RobotState.isEnabled() && !RobotState.isTest())
            .onTrue(
                ClimberCommands
                    .createZeroCommand(this)
                    .onlyIf(() -> climbState.equals(ClimberState.ZEROED))
            );

        // Debug
        SmartDashboard.putData("Deploy Solenoid", Commands.runOnce(this::deploySolenoid));
        SmartDashboard.putData("Disable Solenoid", Commands.runOnce(this::disableSolenoid));

        return this;
    }

    // -- Getters --

    public ClimberState getClimbState() {
        return climbState;
    }

    @AutoLogOutput(key = "Climber/Is at Target Position?")
    public boolean isAtTargetPosition() {
        return (Math.abs(climberMotorPosition.getValueAsDouble() - targetPosition) < ClimberConstants.CLIMB_POSITION_TOLERANCE);
    }

    // -- Climb Control --

    public void stow() {
        targetPosition = ClimberConstants.STOW_POSITION;
        Logger.info("Climber stow");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(0));

        climbState = ClimberState.STOWED;
    }

    public void deploy() {
        targetPosition = ClimberConstants.DEPLOY_POSITION;
        Logger.info("Climber deploy");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(0));

        climbState = ClimberState.DEPLOYED;
    }

    public void climb() {
        targetPosition = ClimberConstants.CLIMB_POSITION;
        Logger.info("Climber climb");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(1));

        climbState = ClimberState.CLIMBED;
    }

    public void deploySolenoid() {
        ratchetSolenoid.set(VictorSPXControlMode.PercentOutput, ClimberConstants.RATCHET_SOLENOID_DEPLOY_PERCENTAGE);
    }

    public void disableSolenoid() {
        Logger.info("Disable solenoid");
        ratchetSolenoid.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void disableClimberMotors() {
        climberMotor.stopMotor();
    }

    public void zeroPosition() {
        climberMotor.setPosition(ClimberConstants.CLIMBER_ZERO_POSITION);
        climberFollower.setPosition(ClimberConstants.CLIMBER_ZERO_POSITION);
    }

    // -- Getters --

    public double getLeadCurrent() {
        return climberMotorCurrent.getValueAsDouble();
    }

    public double getFollowerCurrent() {
        return climberFollowerCurrent.getValueAsDouble();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Climber/", statusSignals);

        solenoidVoltage = ratchetSolenoid.getMotorOutputVoltage();
    }

    // -- States --

    public enum ClimberState {
        STOWED,
        DEPLOYED,
        CLIMBED,
        ZEROED
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(SysIdTests.characterize(
            "Climber",
            this,
            climberMotor,
            Volts.of(0.25).per(Second),
            Volts.of(1),
            climberFollower,
            true
        ));
    }
}