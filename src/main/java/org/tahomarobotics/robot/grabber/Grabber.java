package org.tahomarobotics.robot.grabber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.game.GamePiece;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.grabber.GrabberConstants.*;

public class Grabber extends SubsystemIF {
    private final static Grabber INSTANCE = new Grabber();

    // -- Member Variables --

    // Hardware

    private final TalonFX motor;

    // Status Signals

    private final StatusSignal<Current> current;

    private final LoggedStatusSignal[] statusSignals;

    // Control requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);
    private final VoltageOut voltageControl = new VoltageOut(0);

    // State

    @AutoLogOutput(key = "Grabber/State")
    private GrabberState state = GrabberState.DISABLED;

    final Timer collectionTimer = new Timer();

    // -- Initialization --

    private Grabber() {
        // Create hardware

        motor = new TalonFX(RobotMap.END_EFFECTOR_MOTOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Grabber Motor", motor, motorConfig);

        // Bind status signals

        current = motor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Grabber Velocity", motor.getVelocity()),
            new LoggedStatusSignal("Grabber Current", current)
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    public static Grabber getInstance() {
        return INSTANCE;
    }

    private static final Indexer indexer = Indexer.getInstance();

    // -- State Machine --

    public void setTargetState(GrabberState state) {
        this.state = state;

        switch (state.type) {
            case NONE -> motor.stopMotor();
            case VELOCITY -> motor.setControl(velocityControl.withVelocity(state.value));
            case VOLTAGE -> motor.setControl(voltageControl.withOutput(state.value));
        }
    }

    private void stateMachine() {
        if (state == GrabberState.COLLECTING) {
            if (current.getValueAsDouble() > COLLECTION_CURRENT_THRESHOLD &&
                Collector.getInstance().getCollectionMode() != GamePiece.ALGAE) {
                collectionTimer.start();
            } else {
                collectionTimer.stop();
                collectionTimer.reset();
            }
        }

        if (collectionTimer.hasElapsed(COLLECTION_DELAY)) {
            transitionToHolding();
            indexer.transitionToDisabled();

            collectionTimer.stop();
            collectionTimer.reset();
        }
    }

    // Transitions

    public void transitionToDisabled() {
        if (isHolding()) { return; }
        setTargetState(GrabberState.DISABLED);
    }

    public void transitionToHolding() {
        setTargetState(GrabberState.HOLDING);
    }

    public void transitionToCollecting() {
        if (!isArmAtPassing() || isHolding()) { return; }
        setTargetState(GrabberState.COLLECTING);
    }

    public void transitionToScoring() {
        setTargetState(GrabberState.SCORING);
    }

    // -- Getters --

    public boolean isArmAtPassing() {
        return SmartDashboard.getBoolean("arm at position", true);
    }

    public boolean isHolding() {
        return state == GrabberState.HOLDING;
    }

    public double getCurrent() {
        return current.getValueAsDouble();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Grabber/", statusSignals);

        stateMachine();
    }

    @Override
    public void onDisabledInit() {
        transitionToDisabled();
    }

    // -- Autonomous --

    @Override
    public void onAutonomousInit() {
        transitionToHolding();
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Grabber",
                this,
                motor,
                Volts.of(0.25).per(Second),
                Volts.of(1)
            )
        );
    }
}
