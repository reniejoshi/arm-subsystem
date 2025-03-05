package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.identity.Identity;

public class WindmillConstants {
    // -- Elevator --

    // States

    public enum TrajectoryState {
        HIGH_DESCORE(new Translation2d(-0.4, 0.90)),
        LOW_DESCORE(new Translation2d(-0.4, 0.54)),
        COLLECT(new Translation2d(-0.025, -0.22)),
        STOW(new Translation2d(0, 0.7)),
        L4(new Translation2d(-0.332, 1.59)),
        L3(new Translation2d(-0.369, 0.984)),
        L2(new Translation2d(-0.35, 0.572)),
        START(new Translation2d(0, ARM_LENGTH));

        public final Translation2d t2d;

        TrajectoryState(Translation2d t2d) {
            this.t2d = t2d;
        }
    }

    // Gearing

    public static final double ELEVATOR_GEAR_REDUCTION;

    // Pulley

    public static final double ELEVATOR_MAIN_PULLEY_RADIUS = Units.inchesToMeters(1.1056);
    public static final double ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE = 2 * Math.PI * ELEVATOR_MAIN_PULLEY_RADIUS;

    // Poses

    public static final double ELEVATOR_MAX_POSE = 1.035; // Meters
    public static final double ELEVATOR_MIN_POSE = 0.01; // Meters

    public static final double ELEVATOR_COLLECT_POSE = 0.43;
    public static final double ELEVATOR_HIGH_POSE = 1.03; // Meters
    public static final double ELEVATOR_MID_POSE = 0.45; // Meters
    public static final double ELEVATOR_LOW_POSE = 0.05; // Meters

    // Tolerances

    public static final double ELEVATOR_POSITION_TOLERANCE = 0.005; // Meters
    public static final double ELEVATOR_VELOCITY_TOLERANCE = 0.01; // Meters / sec

    // Motion

    public static final double ELEVATOR_MAX_VELOCITY = 4; // Meters / sec
    public static final double ELEVATOR_MAX_ACCELERATION = ELEVATOR_MAX_VELOCITY * 4; // Meters / sec^2
    public static final double ELEVATOR_MAX_JERK = ELEVATOR_MAX_ACCELERATION * 4.0; // Meters / sec^3

    // -- Arm --

    // Gearing
    public static final double ARM_BELT_REDUCTION = 18d / 72d;
    public static final double ARM_GEAR_REDUCTION;

    // Poses

    public static final double ARM_CALIBRATION_POSE = 0.25;
    public static final double ARM_COLLECT_POSE = 0.75;
    public static final double ARM_UPRIGHT_POSE = 0.25;
    public static final double ARM_TEST_POSE = 0; // TODO: temporary

    // Tolerances

    public static final double ARM_POSITION_TOLERANCE = 0.005; // Rotations
    public static final double ARM_VELOCITY_TOLERANCE = 0.01; // Rotations / sec

    // Motion

    public static final double ARM_MAX_VELOCITY = 1.5; // Rotations / sec
    public static final double ARM_MAX_ACCELERATION = 4; // Rotations / sec^2
    public static final double ARM_MAX_JERK = ARM_MAX_ACCELERATION * 4; // Rotations / sec^3

    // Constants

    public static final double ARM_LENGTH = 0.6540246; // Meters

    public static final double ELEVATOR_ZEROING_VOLTAGE = -1;
    public static final double ELEVATOR_ZEROING_TIMEOUT = 0.2;

    // -- Constraints --

    public static final double END_EFFECTOR_MIN_HEIGHT = -0.25; // Bottom-most point the carriage hits
    public static final double END_EFFECTOR_MAX_HEIGHT = ELEVATOR_MAX_POSE + ARM_LENGTH;

    // -- Identity --

    static {
        switch (Identity.robotID) {
            case BEEF, BEARRACUDA -> {
                ELEVATOR_GEAR_REDUCTION = 10d / 52d;
                ARM_GEAR_REDUCTION = 8d / 60d * 24d / 50d * ARM_BELT_REDUCTION;
            }
            default -> {
                ELEVATOR_GEAR_REDUCTION = 12d / 52d;
                ARM_GEAR_REDUCTION = 10d / 60d * 24d / 50d * ARM_BELT_REDUCTION;
            }
        }
    }

    // -- Configurations --

    static final TalonFXConfiguration elevatorMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                // SysId'd 1/17/2025
                .withKP(50.438)
                .withKI(100)
                .withKS(0.097499)
                .withKV(3.3441)
                .withKA(0.16116)
                .withKG(0.084958)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(false)
                .withSupplyCurrentLimitEnable(false)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ELEVATOR_MAX_VELOCITY)
                .withMotionMagicAcceleration(ELEVATOR_MAX_ACCELERATION)
                .withMotionMagicJerk(ELEVATOR_MAX_JERK)
        ).withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs()
                .withContinuousWrap(false)
        ).withFeedback(new FeedbackConfigs()
                           .withSensorToMechanismRatio(1 / ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE)
                           .withFeedbackRemoteSensorID(RobotMap.ELEVATOR_ENCODER)
                           .withFeedbackSensorSource(
                               RobotConfiguration.RIO_PHOENIX_PRO ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RemoteCANcoder)
        ).withAudio(
            new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true)
        );

    static final TalonFXConfiguration armMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                // Hand-tuned 02/06/25
                .withKP(80)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0)
                .withKG(0.4)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ARM_MAX_VELOCITY)
                .withMotionMagicAcceleration(ARM_MAX_ACCELERATION)
                .withMotionMagicJerk(ARM_MAX_JERK)
        ).withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs()
                .withContinuousWrap(true)
        ).withFeedback(new FeedbackConfigs()
                           .withSensorToMechanismRatio(1 / ARM_BELT_REDUCTION)
                           .withFeedbackRemoteSensorID(RobotMap.ARM_ENCODER)
                           .withFeedbackSensorSource(
                               RobotConfiguration.RIO_PHOENIX_PRO ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RemoteCANcoder)
        ).withAudio(
            new AudioConfigs()
                .withBeepOnBoot(true)
                .withBeepOnConfig(true)
        );

    static final CANcoderConfiguration armEncoderConfiguration = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
                              .withAbsoluteSensorDiscontinuityPoint(1)
                              .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        );
}
