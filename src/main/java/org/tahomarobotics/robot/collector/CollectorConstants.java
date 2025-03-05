package org.tahomarobotics.robot.collector;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.util.identity.Identity;


public class CollectorConstants {
    // Gearing

    private static final double DEPLOYMENT_REDUCTION = (9d / 84d) * (12d / 36d);

    // Motion Magic Constraints

    private static final double DEPLOYMENT_MAX_VELOCITY = 3;
    private static final double DEPLOYMENT_MAX_ACCELERATION = DEPLOYMENT_MAX_VELOCITY * 4;
    private static final double DEPLOYMENT_MAX_JERK = DEPLOYMENT_MAX_ACCELERATION * 4;

    private static final double COLLECTOR_MAX_VELOCITY = 50.0;
    private static final double COLLECTOR_MAX_ACCELERATION = COLLECTOR_MAX_VELOCITY * 4;
    private static final double COLLECTOR_MAX_JERK = COLLECTOR_MAX_ACCELERATION * 4;

    // Current Limits

    private static final double COLLECTOR_MAX_SUPPLY_CURRENT_LIMIT = 50;

    // Zeroing

    public static final double DEPLOYMENT_ZERO_POSITION;

    public static final double DEPLOYMENT_ZEROING_TIMEOUT = 5;
    public static final double DEPLOYMENT_ZEROING_VOLTAGE = 1;

    public static final double DEPLOYMENT_AT_POSITION_THRESHOLD = 0.005;
    public static final double DEPLOYMENT_MOVING_VELOCITY_THRESHOLD = 0.001;

    // States

    public static final double ALGAE_HOLDING_CURRENT_THRESHOLD = 30;
    private static final double ALGAE_HOLDING_VOLTAGE = 2.5;

    // -- Identity --

    static {
        switch (Identity.robotID) {
            case BEEF, BEARRACUDA -> DEPLOYMENT_ZERO_POSITION = 103.75;
            default -> DEPLOYMENT_ZERO_POSITION = 83.75;
        }
    }

    /**
     * NOTE: The collector rollers do not all run at the same velocity,
     * so speeds are for the motor itself.
     */
    public enum TargetCollectorState {
        DISABLED(),
        COLLECTING(MotionType.VELOCITY, COLLECTOR_MAX_VELOCITY),
        HOLDING_ALGAE(MotionType.VOLTAGE, ALGAE_HOLDING_VOLTAGE),
        EJECTING(MotionType.VELOCITY, -COLLECTOR_MAX_VELOCITY);

        public final MotionType type;
        public final double value;

        TargetCollectorState() {
            this(MotionType.NONE, 0);
        }

        TargetCollectorState(MotionType type, double value) {
            this.type = type;
            this.value = value;
        }

        public enum MotionType {
            VELOCITY, VOLTAGE, NONE
        }
    }

    /**
     * - Looking from the left of the robot, + is CW. <p>
     * - Zero: 19.6deg from horizontal (CoG inline with pivot) <p>
     * - Range of Motion: (~83, ~-21deg) from horizontal <p>
     */
    public enum TargetDeploymentState {
        ZEROED(Units.degreesToRotations(DEPLOYMENT_ZERO_POSITION)),
        STOW(Units.degreesToRotations(75)),
        CORAL_COLLECT(Units.degreesToRotations(-20.75)),
        ALGAE_COLLECT(Units.degreesToRotations(0)),
        EJECT(Units.degreesToRotations(-20.75));

        /** Angle of the deployment motors in rotations for this state. */
        public final double angle;

        TargetDeploymentState(double angle) {
            this.angle = angle;
        }
    }

    // Configurations

    // TODO: Add current limits that are higher than operating currents to not cook motors at edge cases (collisions, etc.).

    public static final TalonFXConfiguration deploymentLeftMotorConfiguration = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        ).withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(1 / DEPLOYMENT_REDUCTION)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(DEPLOYMENT_MAX_VELOCITY)
                .withMotionMagicAcceleration(DEPLOYMENT_MAX_ACCELERATION)
                .withMotionMagicJerk(DEPLOYMENT_MAX_JERK)
        ).withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(50)
                .withKG(0.25)
        ).withAudio(
            new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true)
        );

    public static final MotorOutputConfigs deploymentRightMotorOutputConfiguration = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);

    public static final TalonFXConfiguration collectorMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(COLLECTOR_MAX_ACCELERATION)
                .withMotionMagicJerk(COLLECTOR_MAX_JERK)
        ).withSlot0(
            new Slot0Configs()
                .withKP(0.049)
                .withKS(0.29918)
                .withKV(0.12028)
                .withKA(0.0029652)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(COLLECTOR_MAX_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(false)
        ).withAudio(
            new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true)
        );
}