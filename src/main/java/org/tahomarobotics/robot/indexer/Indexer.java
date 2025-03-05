package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.indexer.IndexerConstants.IndexerState;
import static org.tahomarobotics.robot.indexer.IndexerConstants.configuration;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    // -- Member Variables --

    // Hardware

    private final TalonFX motor;

    private final DigitalInput beanBake;

    // Status Signals

    private final StatusSignal<Current> current;

    private final LoggedStatusSignal[] statusSignals;

    // Control Requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

    // State

    @AutoLogOutput(key = "Indexer/State")
    private IndexerState state = IndexerState.DISABLED;

    // -- Initialization --

    private Indexer() {
        // Create hardware

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);

        beanBake = new DigitalInput(RobotMap.BEAM_BREAK);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Indexer Motor", motor, configuration);

        // Bind status signals

        current = motor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Position", motor.getPosition()),
            new LoggedStatusSignal("Velocity", motor.getVelocity()),
            new LoggedStatusSignal("Current", current)
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    // -- State Machine --

    private void setTargetState(IndexerState state) {
        this.state = state;
        motor.setControl(velocityControl.withVelocity(state.velocity));
    }

    private void stateMachine() {
        if (state == IndexerState.COLLECTING && isBeanBakeTripped()) {
            transitionToPassing();
        }
    }

    // Transitions

    public void transitionToDisabled() {
        setTargetState(IndexerState.DISABLED);
    }

    public void transitionToCollecting() {
        setTargetState(IndexerState.COLLECTING);
    }

    public void transitionToPassing() {
        setTargetState(IndexerState.PASSING);
    }

    public void transitionToEjecting() {
        setTargetState(IndexerState.EJECTING);
    }

    // -- Getter(s) --

    public double getCurrent() {
        return current.getValueAsDouble();
    }

    @AutoLogOutput(key = "Indexer/Beam Brake Tripped?")
    public boolean isBeanBakeTripped() {
        return !beanBake.get();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Indexer/", statusSignals);

        stateMachine();
    }

    @Override
    public void onDisabledInit() {
        transitionToDisabled();
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Indexer SysId Test",
                this,
                motor,
                Volts.of(1).per(Second),
                Volts.of(3)
            ));
    }

}


