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

package org.tahomarobotics.robot.windmill;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.identity.Identity;

public final class WindmillConstants {

    // -- Elevator --

    // States

    public enum TrajectoryState {
        HIGH_DESCORE(0.630, Units.degreesToRadians(130.693),
                     Units.inchesToMeters(15.257), Units.degreesToRadians(127.820)),  // ALGAE: B button
        LOW_DESCORE(0.254, Units.degreesToRadians(133.945),
                    Units.inchesToMeters(0.815), Units.degreesToRadians(127.742)),  // ALGAE: A button
        CORAL_COLLECT(0.430, Units.degreesToRadians(-92.000),
                      Units.inchesToMeters(17.069), Units.degreesToRadians(-92.191)),  // CORAL: RS button toggle
        STOW(Units.inchesToMeters(1.901), Units.degreesToRadians(89.913),
             Units.inchesToMeters(1.901), Units.degreesToRadians(89.913)),  // RS button toggle
        ALGAE_COLLECT(0.0214, Units.degreesToRadians(177.211),
                      0.0214, Units.degreesToRadians(177.211)),  // ALGAE: RS button toggle
        ALGAE_PASSOFF(Units.inchesToMeters(42.344), Units.degreesToRadians(-72.191),
                      Units.inchesToMeters(42.344), Units.degreesToRadians(-72.191)),
        ALGAE_PROCESSOR(0.567, Units.degreesToRadians(-61.566),
                        0.567, Units.degreesToRadians(-61.566)),  // ALGAE: X button
        ALGAE_PRESCORE(Units.inchesToMeters(39.244), Units.degreesToRadians(-60),
                       Units.inchesToMeters(39.244), Units.degreesToRadians(-60)),  // ALGAE: Y button
        ALGAE_SCORE(Units.inchesToMeters(42.344), Units.degreesToRadians(0),
                    Units.inchesToMeters(42.344), Units.degreesToRadians(0)),  // ALGAE: Y button
        L4(Units.inchesToMeters(42.344), Units.degreesToRadians(120.548),
           Units.inchesToMeters(40.344), Units.degreesToRadians(120.548)),  // CORAL: Y button
        L3(0.479, Units.degreesToRadians(125.551),
           Units.inchesToMeters(17.500), Units.degreesToRadians(124.398)),  // CORAL: B button
        L2(0.07375, Units.degreesToRadians(122.539),
           Units.inchesToMeters(1.364), Units.degreesToRadians(124.791)),  // CORAL: A button
        L1(0.572, Units.degreesToRadians(-121.465),
           0.572, Units.degreesToRadians(-121.465)),  // CORAL: X button
        START(Units.inchesToMeters(0.468), Units.degreesToRadians(90.089),
              Units.inchesToMeters(0.468), Units.degreesToRadians(90.089));  // startup only

        public final double elev;
        public final double arm;
        public final WindmillState state;

        TrajectoryState(double elevAEE, double armAEE, double elevSEE, double armSEE) {
            this.elev = (RobotConfiguration.FEATURE_ALGAE_END_EFFECTOR) ? elevAEE : elevSEE;
            this.arm = (RobotConfiguration.FEATURE_ALGAE_END_EFFECTOR) ? armAEE : armSEE;
            this.state = new WindmillState(
                0,
                new WindmillState.ElevatorState(elev, 0, 0),
                new WindmillState.ArmState(arm, 0, 0)
            );
        }

        public boolean collectsAlgae() {
            return this == ALGAE_COLLECT || this == HIGH_DESCORE || this == LOW_DESCORE;
        }

        public boolean shouldAutoScore() {
            return this == L2 || this == L3 || this == L4;
        }
    }

    // obstruction thresholds
    public static final double ROBOT_DECK = Units.inchesToMeters(-9.0);
    public static final double ROBOT_COLLECTOR_X = Units.inchesToMeters(0.0);
    public static final double ROBOT_COLLECTOR_Y = Units.inchesToMeters(-0.5);

    public static final double SMALL_PULLBACK = Units.degreesToRadians(10.0);
    public static final double STANDARD_PULLBACK = Units.degreesToRadians(20.0);
    public static final double LARGE_PULLBACK = Units.degreesToRadians(30.0);
    public static final double ALGAE_PULLBACK = Units.degreesToRadians(25.0);

    // Gearing
    public static final double ELEVATOR_GEAR_REDUCTION;

    // Pulley
    public static final double ELEVATOR_MAIN_PULLEY_RADIUS = Units.inchesToMeters(1.1056);
    public static final double ELEVATOR_MAIN_PULLEY_CIRCUMFERENCE = 2 * Math.PI * ELEVATOR_MAIN_PULLEY_RADIUS;

    // Poses
    public static final double ELEVATOR_MAX_POSE = 1.035 + Units.inchesToMeters(6); // Meters
    public static final double ELEVATOR_MIN_POSE = 0.01; // Meters

    public static final double ELEVATOR_MID_POSE = 0.45; // Meters

    // Tolerances

    public static final double ELEVATOR_POSITION_TOLERANCE = 0.005; // Meters
    public static final double ELEVATOR_VELOCITY_TOLERANCE = 0.01; // Meters / sec

    // Motion

    public static final double ELEVATOR_MAX_VELOCITY = 2.0; // Meters / sec
    public static final double ELEVATOR_MAX_ACCELERATION = 15.0; // Meters / sec^2
    public static final double ELEVATOR_MAX_JERK = 200; // Meters / sec^3

    // -- Arm --

    // Gearing
    public static final double ARM_BELT_REDUCTION = 18d / 72d;
    public static final double ARM_ROTOR_TO_ENCODER;

    // Poses

    public static final double ARM_CALIBRATION_POSE = 0.25; // Rotation (used for motor only)

    // Tolerances

    public static final double ARM_POSITION_TOLERANCE = Units.rotationsToRadians(0.005); // Radians
    public static final double ARM_VELOCITY_TOLERANCE = Units.rotationsToRadians(0.01); // Radians / sec

    // Motion
    public static final double ARM_MAX_VELOCITY = 1.75 * Math.PI; // Radians / sec
    public static final double ARM_MAX_ACCELERATION = 15.0 * Math.PI; // Radians / sec^2
    public static final double ARM_MAX_JERK = 200 * Math.PI;  // Radians / sec^3

    public static final double ARM_ALGAE_MAX_VELOCITY = ARM_MAX_VELOCITY * 0.3;
    public static final double ARM_ALGAE_MAX_ACCELERATION = ARM_MAX_ACCELERATION * 0.3;

    public static final double ARM_ALGAE_THROW_VELOCITY = 1.5 * Math.PI;
    public static final double ARM_ALGAE_THROW_ACCELERATION = 1.0 * Math.PI;
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
            case BEEF -> {
                ELEVATOR_GEAR_REDUCTION = 10d / 52d;
                ARM_ROTOR_TO_ENCODER = 8d / 60d * 24d / 50d;
            }
            case BEARRACUDA -> {
                ELEVATOR_GEAR_REDUCTION = 12d / 52d;
                ARM_ROTOR_TO_ENCODER = 8d / 60d * 24d / 50d;
            }
            default -> {
                ELEVATOR_GEAR_REDUCTION = 12d / 52d;
                ARM_ROTOR_TO_ENCODER = 10d / 60d * 24d / 50d;
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
                           .withRotorToSensorRatio(1 / ELEVATOR_GEAR_REDUCTION)
                           .withFeedbackRemoteSensorID(RobotMap.ELEVATOR_ENCODER)
                           .withFeedbackSensorSource(
                               RobotConfiguration.WINDMILL_PHOENIX_PRO ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RotorSensor)
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
                .withMotionMagicCruiseVelocity(Units.radiansToRotations(ARM_MAX_VELOCITY))
                .withMotionMagicAcceleration(Units.radiansToRotations(ARM_MAX_ACCELERATION))
                .withMotionMagicJerk(Units.radiansToRotations(ARM_MAX_JERK))
        ).withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs()
                .withContinuousWrap(true)
        ).withFeedback(new FeedbackConfigs()
                           .withSensorToMechanismRatio(1 / ARM_BELT_REDUCTION)
                           .withRotorToSensorRatio(1 / ARM_ROTOR_TO_ENCODER)
                           .withFeedbackRemoteSensorID(RobotMap.ARM_ENCODER)
                           .withFeedbackSensorSource(
                               RobotConfiguration.WINDMILL_PHOENIX_PRO ? FeedbackSensorSourceValue.FusedCANcoder : FeedbackSensorSourceValue.RotorSensor)
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
