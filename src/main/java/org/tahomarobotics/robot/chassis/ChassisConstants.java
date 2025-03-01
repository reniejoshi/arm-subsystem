package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;

import java.util.Objects;

@SuppressWarnings("SuspiciousNameCombination")
public class ChassisConstants {
    // Physical

    /** Approximate width of the robot with bumpers. */
    public static final double BUMPER_WIDTH = Units.inchesToMeters(34);

    /** Width-wise distance between wheel centers in <strong>meters</strong> */
    private static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
    /** Length-wise distance between wheel centers in <strong>meters</strong> */
    private static final double WHEELBASE = Units.inchesToMeters(20.75);

    public static final double HALF_TRACK_WIDTH = TRACK_WIDTH / 2;
    public static final double HALF_WHEELBASE = WHEELBASE / 2;

    /** Approximate mass of the robot in <strong>kilograms</strong> */
    private static final double BATTERY_MASS = Units.lbsToKilograms(13.6);
    private static final double BUMPER_MASS = Units.lbsToKilograms(16.0);
    private static final double ROBOT_MASS = Units.lbsToKilograms(119.0);
    public static final double MASS = ROBOT_MASS + BATTERY_MASS + BUMPER_MASS;

    /** Approximate radius of the wheel in <strong>meters</strong> */
    private static final double WHEEL_RADIUS = Units.inchesToMeters(3.95 / 2 - 0.1);
    /** Approximate circumference of the wheel in <strong>meters</strong> */
    private static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    /** Approximate Moment of Inertia based on CAD in <strong>kilogram meter squared</strong>. */
    // https://www.swervedrivespecialties.com/collections/mk4i-parts/products/billet-wheel-4d-x-1-5w-bearing-bore
    public static final double WHEEL_MOI = 0.00031307;

    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(HALF_WHEELBASE, -HALF_TRACK_WIDTH);
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-HALF_WHEELBASE, -HALF_TRACK_WIDTH);

    // Gearing

    /** Reduction between the drive motor and the wheel. */
    public static final double DRIVE_REDUCTION = Gearing.L1_PLUS.driveReduction;
    /** Ratio between angular position and position in <strong>meters</strong> for the wheel. */
    public static final double DRIVE_POSITION_COEFFICIENT = WHEEL_CIRCUMFERENCE * DRIVE_REDUCTION;

    public static final double ROBOT_MOI = 1.0 / 12.0 * MASS *  (TRACK_WIDTH * TRACK_WIDTH + WHEELBASE * WHEELBASE);

    /**
     * Load inertia for each drive motor in <strong>kilogram meter squared</strong>.
     * <p>
     * {@code J_load = M / N * (r / G)^2} where M is the mass of the robot,
     * N is the number of modules, r is the radius of the wheel, and G is the gear ratio.
     */
    public static final double DRIVE_MOTOR_MOI = MASS / 4 * Math.pow(WHEEL_RADIUS * DRIVE_REDUCTION, 2);
    /** Coefficient of friction between the wheel and the ground. */
    public static final double WHEEL_COF = 1.0; // Placeholder
    // Simulation

    /** A semi-arbitrary scaling factor for the motor model MOIs. */
    public static final double MOI_SCALING_FACTOR = 10;

    // Motion

    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
    /** Theoretical max velocity of the robot in <strong>meters per second</strong>. */
    public static final double MAX_VELOCITY = DRIVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION * WHEEL_RADIUS;
    /** Theoretical max angular velocity of the robot in <strong>meters per second</strong>. */
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE);
    /** Max average acceleration across all modules in <strong>meters per second squared</strong>. */
    public static final double ACCELERATION_LIMIT = 6.0;
    //Placeholder PID values
    public static final PIDConstants AUTO_TRANSLATION_PID = new PIDConstants(7.5,0,0.5);
    public static final PIDConstants AUTO_ROTATION_PID = new PIDConstants(5, 0, 0.5);

    // Control Loops

    @SuppressWarnings("HungarianNotationConstants")
    public static final double kV_DRIVE = (2 * Math.PI) / DRIVE_MOTOR.KvRadPerSecPerVolt;

    // Current Limits

    private static final double DRIVE_SUPPLY_CURRENT_LIMIT = 50;
    private static final double DRIVE_STATOR_CURRENT_LIMIT = 100;
    private static final double STEER_SUPPLY_CURRENT_LIMIT = 20;
    private static final double STEER_STATOR_CURRENT_LIMIT = 30;

    // Configuration

    public static final RobotConfig robotConfig = new RobotConfig(
        MASS,
        ROBOT_MOI,
        new ModuleConfig(
            WHEEL_RADIUS,
            MAX_VELOCITY * 0.95,
            WHEEL_COF,
            DRIVE_MOTOR.withReduction(1.0/DRIVE_REDUCTION),
            DRIVE_SUPPLY_CURRENT_LIMIT,  // This is used when calculating current available at Nominal Voltage (@12V) which is limited via the supply current
            1
        ),
        FRONT_LEFT_OFFSET,
        FRONT_RIGHT_OFFSET,
        BACK_LEFT_OFFSET,
        BACK_RIGHT_OFFSET
    );

    /** @return The configuration for the drive motor. */
    public static TalonFXConfiguration createDriveMotorConfiguration() {
        return new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.15)
                    .withKV(kV_DRIVE)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive)
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(DRIVE_SUPPLY_CURRENT_LIMIT)
            ).withAudio(
                new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true)
                    .withAllowMusicDurDisable(true)
            );
    }

    /** @return The configuration for the steer motor. */
    public static TalonFXConfiguration createSteerMotorConfiguration(String moduleName, int encoderId) {
        // TODO: This sucks, but is the easiest way to do this right now.
        double steerReduction = Objects.equals(
            RobotMap.BACK_LEFT_MOD.moduleName(), moduleName) ? Type.MK4n.steerReduction : Type.MK4i.steerReduction;

        return new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(8.0)
                    .withKI(0.01)
                    .withKD(0.16)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive)
            ).withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{
                ContinuousWrap = true;
            }}).withFeedback(
                new FeedbackConfigs() {{
                    FeedbackRemoteSensorID = encoderId;

                    if (RobotConfiguration.CANIVORE_PHOENIX_PRO) {
                        FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                        RotorToSensorRatio = 1 / steerReduction;
                    } else {
                        FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                    }
                }}
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT)
                    .withSupplyCurrentLimit(STEER_SUPPLY_CURRENT_LIMIT)
            ).withAudio(
                new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true)
                    .withAllowMusicDurDisable(true)
            );
    }

    public static CANcoderConfiguration createEncoderConfiguration(double angularOffset) {
        return new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(1)
                    .withMagnetOffset(angularOffset)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            );
    }

    // Vendor

    public enum Type {
        MK4i((14.0 / 50.0) * (10.0 / 60.0)),
        MK4n(1 / 18.75);

        final double steerReduction;

        Type(double steerReduction) {
            this.steerReduction = steerReduction;
        }
    }

    public enum Gearing {
        L1_PLUS((16.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)),
        L2_PLUS((16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));

        final double driveReduction;

        Gearing(double driveReduction) {
            this.driveReduction = driveReduction;
        }
    }
}
