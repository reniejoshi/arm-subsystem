package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.game.GamePiece;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tinylog.Logger;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.collector.CollectorConstants.*;

public class Collector extends SubsystemIF {
    private final static Collector INSTANCE = new Collector();

    // -- Member Variables --

    // Hardware

    private final TalonFX leftMotor; // Leader
    private final TalonFX rightMotor; // Follower
    private final TalonFX collectorMotor;

    // Status Signals

    private final StatusSignal<Angle> leftDeploymentPosition;
    private final StatusSignal<AngularVelocity> leftDeploymentVelocity, rightDeploymentVelocity;
    private final StatusSignal<Current> collectorCurrent, leftDeploymentCurrent, rightDeploymentCurrent;

    private final LoggedStatusSignal[] statusSignals;

    // Control Requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0.0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);

    private final VoltageOut voltageControl = new VoltageOut(0.0);

    // State

    @AutoLogOutput(key = "Collector/Target Deploy State")
    private TargetDeploymentState targetDeploymentState = TargetDeploymentState.ZEROED;
    @AutoLogOutput(key = "Collector/Target Collector State")
    private TargetCollectorState targetCollectorState = TargetCollectorState.DISABLED;
    @AutoLogOutput(key = "Game Piece Mode")
    private GamePiece collectionMode = GamePiece.CORAL;

    // -- Initialization --

    private Collector() {
        // Create hardware

        leftMotor = new TalonFX(RobotMap.COLLECTOR_LEFT_MOTOR);
        rightMotor = new TalonFX(RobotMap.COLLECTOR_RIGHT_MOTOR);
        collectorMotor = new TalonFX(RobotMap.COLLECTOR_COLLECT_MOTOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Collector Left Motor", leftMotor, deploymentLeftMotorConfiguration);
        RobustConfigurator.tryConfigureTalonFX(
            "Collector Right Motor", rightMotor,
            deploymentLeftMotorConfiguration.withMotorOutput(deploymentRightMotorOutputConfiguration)
        );
        RobustConfigurator.tryConfigureTalonFX("Collector Collect Motor", collectorMotor, collectorMotorConfig);

        // Bind status signals

        leftDeploymentPosition = leftMotor.getPosition();
        leftDeploymentVelocity = leftMotor.getVelocity();
        rightDeploymentVelocity = rightMotor.getVelocity();

        leftDeploymentCurrent = leftMotor.getSupplyCurrent();
        rightDeploymentCurrent = rightMotor.getSupplyCurrent();
        collectorCurrent = collectorMotor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Left Deployment Position", leftDeploymentPosition),
            new LoggedStatusSignal("Right Deployment Position", rightMotor.getPosition()),
            new LoggedStatusSignal("Left Deployment Velocity", leftDeploymentVelocity),
            new LoggedStatusSignal("Right Deployment Velocity", rightDeploymentVelocity),
            new LoggedStatusSignal("Collector Velocity", collectorMotor.getVelocity()),
            new LoggedStatusSignal("Left Deployment Current", leftDeploymentCurrent),
            new LoggedStatusSignal("Right Deployment Current", rightDeploymentCurrent),
            new LoggedStatusSignal("Collector Collect Current", collectorCurrent)
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor, collectorMotor);
    }

    public static Collector getInstance() {
        return INSTANCE;
    }

    // -- Zeroing --

    public void setZeroingVoltage() {
        leftMotor.setVoltage(DEPLOYMENT_ZEROING_VOLTAGE);
        rightMotor.setVoltage(DEPLOYMENT_ZEROING_VOLTAGE);
    }

    public boolean isDeploymentStopped() {
        return leftDeploymentVelocity.getValueAsDouble() < DEPLOYMENT_MOVING_VELOCITY_THRESHOLD &&
               rightDeploymentVelocity.getValueAsDouble() < DEPLOYMENT_MOVING_VELOCITY_THRESHOLD;
    }

    public void zero() {
        if (RobotState.isDisabled()) { return; }
        Logger.info("Zeroed collector.");

        leftMotor.setPosition(TargetDeploymentState.ZEROED.angle);
        rightMotor.setPosition(TargetDeploymentState.ZEROED.angle);

        targetDeploymentState = TargetDeploymentState.STOW;
        setDeploymentControl(targetDeploymentState);
    }

    // -- Deployment State Machine --

    public TargetDeploymentState getTargetDeploymentState() {
        return targetDeploymentState;
    }

    public boolean isAtTargetDeploymentState() {
        return Math.abs(targetDeploymentState.angle - leftDeploymentPosition.getValueAsDouble()) < DEPLOYMENT_AT_POSITION_THRESHOLD && isDeploymentStopped();
    }

    private void setDeploymentControl(TargetDeploymentState state) {
        leftMotor.setControl(positionControl.withPosition(state.angle));
        rightMotor.setControl(positionControl);
    }

    /**
     * Sync the applied control request for the deployment motors to the target state.
     */
    private void syncDeploymentControl() {
        // Ensure we transition to the correct mode.
        if (isDeploymentCollecting()) {
            targetDeploymentState = collectionMode == GamePiece.CORAL ?
                TargetDeploymentState.CORAL_COLLECT :
                TargetDeploymentState.ALGAE_COLLECT;
        }

        setDeploymentControl(targetDeploymentState);
    }

    // Transitions

    private void setTargetDeploymentState(TargetDeploymentState state) {
        if (checkZeroingGuard()) { return; }

        targetDeploymentState = state;
        syncDeploymentControl();
    }

    public void deploymentTransitionToStow() {
        setTargetDeploymentState(TargetDeploymentState.STOW);

        if (isNotHoldingAlgae()) {
            setTargetCollectorState(TargetCollectorState.DISABLED);
        }
    }

    public void deploymentTransitionToCollect() {
        // Sets the target state depending on the collection mode;
        // Is unnecessary due to syncDeploymentControl, but kept for readability.
        setTargetDeploymentState(
            collectionMode == GamePiece.CORAL ?
                TargetDeploymentState.CORAL_COLLECT :
                TargetDeploymentState.ALGAE_COLLECT
        );
    }

    private void deploymentTransitionToEjecting() {
        if (targetDeploymentState != TargetDeploymentState.STOW) {
            // Overrides the control request while retaining the previous target state.
            setDeploymentControl(TargetDeploymentState.EJECT);
        }
    }

    private void deploymentCancelEjecting() {
        syncDeploymentControl();
    }

    // Checks

    public boolean isDeploymentStowed() {
        return targetDeploymentState == TargetDeploymentState.STOW;
    }

    public boolean isDeploymentCollecting() {
        return targetDeploymentState == TargetDeploymentState.CORAL_COLLECT || targetDeploymentState == TargetDeploymentState.ALGAE_COLLECT;
    }

    public boolean isNotHoldingAlgae() {
        return targetCollectorState != TargetCollectorState.HOLDING_ALGAE;
    }

    // -- Collector State Machine --

    public TargetCollectorState getTargetCollectorState() {
        return targetCollectorState;
    }

    private void collectorStateMachine() {
        if (targetCollectorState == TargetCollectorState.COLLECTING) {
            if (collectorCurrent.getValueAsDouble() > ALGAE_HOLDING_CURRENT_THRESHOLD && collectionMode == GamePiece.ALGAE) {
                collectorTransitionToHolding();
            }
        }
    }

    private void setTargetCollectorState(TargetCollectorState state) {
        if (checkZeroingGuard()) { return; }

        targetCollectorState = state;
        switch (state.type) {
            case VELOCITY -> collectorMotor.setControl(velocityControl.withVelocity(state.value));
            case VOLTAGE -> collectorMotor.setControl(voltageControl.withOutput(state.value));
            case NONE -> collectorMotor.stopMotor();
        }
    }

    // Transitions

    public void collectorTransitionToDisabled() {
        setTargetCollectorState(TargetCollectorState.DISABLED);
    }

    public void collectorTransitionToHolding() {
        setTargetCollectorState(TargetCollectorState.HOLDING_ALGAE);
    }

    public void collectorTransitionToCollecting() {
        setTargetCollectorState(TargetCollectorState.COLLECTING);
    }

    private void collectorTransitionToEjecting() {
        setTargetCollectorState(TargetCollectorState.EJECTING);
    }

    private void collectorCancelEjecting() {
        setTargetCollectorState(TargetCollectorState.DISABLED);
    }

    // -- Combined Transitions --

    public void transitionToEjecting() {
        deploymentTransitionToEjecting();
        collectorTransitionToEjecting();
    }

    public void cancelEjecting() {
        deploymentCancelEjecting();
        collectorCancelEjecting();
    }

    // -- Getters --

    public double getCollectorCurrent() {
        return collectorCurrent.getValueAsDouble();
    }

    public double getLeftDeploymentCurrent() {
        return leftDeploymentCurrent.getValueAsDouble();
    }

    public double getRightDeploymentCurrent() {
        return rightDeploymentCurrent.getValueAsDouble();
    }

    // -- Subsystem Overrides --

    @Override
    public SubsystemIF initialize() {
        new Trigger(() -> RobotState.isEnabled() && !RobotState.isAutonomous())
            .onTrue(
                CollectorCommands
                    .createZeroCommand(this)
                    .onlyIf(() -> targetDeploymentState == TargetDeploymentState.ZEROED)
            );

        return this;
    }

    @Override
    public void periodic() {
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Collector/", statusSignals);

        collectorStateMachine();
    }

    @Override
    public void onDisabledInit() {
        if (targetDeploymentState != TargetDeploymentState.ZEROED) {
            deploymentTransitionToStow();
            collectorTransitionToDisabled();
        }
    }

    // -- Collection Mode --

    public GamePiece getCollectionMode() {
        return collectionMode;
    }

    public void toggleCollectionMode() {
        setCollectionMode(collectionMode == GamePiece.CORAL ? GamePiece.ALGAE : GamePiece.CORAL);
    }

    public void setCollectionMode(GamePiece collectionMode) {
        this.collectionMode = collectionMode;
        syncDeploymentControl();
    }

    // -- Helper Methods --

    private boolean checkZeroingGuard() {
        if (targetDeploymentState == TargetDeploymentState.ZEROED) {
            Logger.error("Attempted to use collector prior to zeroing.");
            return true;
        }
        return false;
    }

    // -- SysId --


    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Collector Deployment",
                this,
                leftMotor,
                Volts.of(0.125).per(Second),
                Volts.of(0.5),
                rightMotor,
                true
            ),
            SysIdTests.characterize(
                "Collector Collector",
                this,
                collectorMotor,
                Volts.of(1).per(Second),
                Volts.of(3)
            )
        );
    }
}