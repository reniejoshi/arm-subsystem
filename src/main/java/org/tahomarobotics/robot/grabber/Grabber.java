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

package org.tahomarobotics.robot.grabber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
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
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;

import java.util.List;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.grabber.GrabberConstants.*;

public class Grabber extends SubsystemIF {
    private final static Grabber INSTANCE = new Grabber();

    // -- Member Variables --

    private final Windmill windmill = Windmill.getInstance();

    // Hardware

    private final TalonFX motor;
    private final CANrange canRange;

    // Status Signals

    private final StatusSignal<Current> current;
    private final StatusSignal<Distance> distance;
    private final StatusSignal<Boolean> detection;

    private final LoggedStatusSignal[] statusSignals;

    // Control requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);
    private final VoltageOut voltageControl = new VoltageOut(0);

    // State

    @AutoLogOutput(key = "Grabber/State")
    private GrabberState state = GrabberState.DISABLED;

    boolean collectingCoral = false;
    final Timer coralCollectionTimer = new Timer();
    final Timer algaeCollectionTimer = new Timer();
    private final Timer belowTimer = new Timer();

    private boolean isUsingSupplier = false;
    private DoubleSupplier velocitySupplier;

    private final Debouncer coralDetectionDebouncer = new Debouncer(CORAL_COLLECTION_DELAY);

    // -- Initialization --

    private Grabber() {
        // Create hardware

        motor = new TalonFX(RobotMap.END_EFFECTOR_MOTOR);
        canRange = new CANrange(RobotMap.RANGE_SENSOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Grabber Motor", motor, motorConfig);
        RobustConfigurator.tryConfigureCANrange("Grabber Range Sensor", canRange, canRangeConfig);

        // Bind status signals

        current = motor.getSupplyCurrent();
        distance = canRange.getDistance();
        detection = canRange.getIsDetected();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Grabber Voltage", motor.getMotorVoltage()),
            new LoggedStatusSignal("Grabber Velocity", motor.getVelocity()),
            new LoggedStatusSignal("Grabber Current", current),
            new LoggedStatusSignal("CANrange Distance", distance),
            new LoggedStatusSignal("CANrange In Range", detection)
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

        isUsingSupplier = state.usingSupplier;

        switch (state.type) {
            case NONE -> motor.stopMotor();
            case POSITION -> motor.setControl(positionControl.withPosition(state.value));
            case VELOCITY -> motor.setControl(velocityControl.withVelocity(state.value));
            case VOLTAGE -> motor.setControl(voltageControl.withOutput(state.value));
        }
    }

    private void stateMachine() {
        if (state == GrabberState.CORAL_COLLECTING) {
            if (!RobotConfiguration.FEATURE_BEAN_BAKE) {
                if (coralDetectionDebouncer.calculate(isInRange())) {
                    transitionToCoralHolding();
                }
            } else {
                if (coralDetectionDebouncer.calculate(indexer.isBeanBakeTripped()) && !collectingCoral) {
                    collectingCoral = true;
                }
                if (!indexer.isBeanBakeTripped() && collectingCoral) {
                    transitionToCoralHolding();
                }
            }
        } else if (state == GrabberState.ALGAE_COLLECTING && RobotConfiguration.FEATURE_ALGAE_END_EFFECTOR) {
            boolean isTripped = getCurrent() > ALGAE_COLLECTION_CURRENT_THRESHOLD;

            if (isTripped && Collector.getInstance().getCollectionMode() != GamePiece.CORAL) {
                algaeCollectionTimer.start();

                belowTimer.reset();
                belowTimer.stop();
            } else if (belowTimer.hasElapsed(0.05)) {
                algaeCollectionTimer.stop();
                algaeCollectionTimer.reset();
            } else {
                belowTimer.restart();
            }
        }

        if (algaeCollectionTimer.hasElapsed(ALGAE_COLLECTION_DELAY) && RobotConfiguration.FEATURE_ALGAE_END_EFFECTOR) {
            transitionToAlgaeHolding();

            Windmill.getInstance().createTransitionCommand(((WindmillConstants.TrajectoryState.STOW))).schedule();

            algaeCollectionTimer.stop();
            algaeCollectionTimer.reset();
        }

        if (coralCollectionTimer.hasElapsed(CORAL_COLLECTION_DELAY)) {
            transitionToCoralHolding();
            indexer.transitionToDisabled();

            collectingCoral = false;
            coralCollectionTimer.stop();
            coralCollectionTimer.reset();
        }
    }

    // Transitions

    public void transitionToDisabled() {
        if (isHoldingCoral()) { return; }
        if (algaeCollectionTimer.isRunning() || collectingCoral) { return; }

        setTargetState(GrabberState.DISABLED);
    }

    public void transitionToCoralHolding() {
        collectingCoral = false;
        setTargetState(GrabberState.CORAL_HOLDING);
    }

    public void transitionToAlgaeHolding() {
        setTargetState(GrabberState.ALGAE_HOLDING);
    }

    public void transitionToAlgaeCollecting() {
        if (!isArmAtPassing() || isHoldingCoral()) { return; }
        setTargetState(GrabberState.ALGAE_COLLECTING);
    }

    public void transitionToCoralCollecting() {
        if (!isArmAtPassing() || isHoldingCoral()) { return; }
        setTargetState(GrabberState.CORAL_COLLECTING);
    }

    public void transitionToScoring() {
        setTargetState(GrabberState.AUTO_SCORING);

        collectingCoral = false;
        coralCollectionTimer.stop();
        coralCollectionTimer.reset();
        algaeCollectionTimer.stop();
        algaeCollectionTimer.reset();
    }

    public void transitionToManualScoring() {
        setTargetState(GrabberState.MANUAL_SCORING);

        collectingCoral = false;
        coralCollectionTimer.stop();
        coralCollectionTimer.reset();
        algaeCollectionTimer.stop();
        algaeCollectionTimer.reset();
    }

    public void transitionToScoringL1() {
        setTargetState(GrabberState.L1_SCORING);
    }

    public void setVelocitySupplier(DoubleSupplier velocitySupplier) {
        this.velocitySupplier = velocitySupplier;
    }

    // -- Getters --

    public boolean isArmAtPassing() {
        return SmartDashboard.getBoolean("arm at position", true);
    }

    public boolean isHoldingCoral() {
        return state == GrabberState.CORAL_HOLDING;
    }

    public boolean isHoldingAlgae() {
        return state == GrabberState.ALGAE_HOLDING;
    }

    public double getCurrent() {
        return current.getValueAsDouble();
    }

    public boolean isScoringL1() {
        return state == GrabberState.L1_SCORING;
    }

    public boolean isScoring() {
        return state == GrabberState.MANUAL_SCORING
            || state == GrabberState.AUTO_SCORING
            || state == GrabberState.L1_SCORING;
    }

    public GrabberState getState() {
        return state;
    }

    @AutoLogOutput(key="Grabber/Is Coral Detected?")
    public boolean isCoralDetected() {
        return state == GrabberState.CORAL_COLLECTING
               && windmill.getTargetTrajectoryState() == WindmillConstants.TrajectoryState.CORAL_COLLECT
               && !windmill.isRunningTrajectory()
               && isInRange();
    }

    public double getRange() {
        return distance.getValueAsDouble();
    }

    @AutoLogOutput(key="Grabber/Is CANRange Tripped?")
    public boolean isInRange() {
        return Math.abs(getRange() - CORAL_CANRANGE_DISTANCE) < CORAL_CANRANGE_DISTANCE_TOLERANCE;
    }

    // -- Periodic --

    @Override
    public void periodic() {
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Grabber/", statusSignals);

        stateMachine();

        if (isUsingSupplier) {
            motor.setControl(velocityControl.withVelocity(state.value * velocitySupplier.getAsDouble()));
        }
    }

    @Override
    public void onDisabledInit() {
        transitionToDisabled();
    }

    // -- Autonomous --

    @Override
    public void onAutonomousInit() {
        transitionToCoralHolding();
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Grabber",
                this,
                motor,
                Volts.of(1).per(Second),
                Volts.of(3)
            )
        );
    }
}