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

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.util.identity.Identity;

public class GrabberConstants {
    public static final double CORAL_COLLECT_VELOCITY = RobotConfiguration.FEATURE_ALGAE_END_EFFECTOR ? -48 : -20;
    public static final double ALGAE_COLLECT_VELOCITY = -10;
    public static final double SCORING_VELOCITY = 50;
    public static final double L1_SCORING_VELOCITY = SCORING_VELOCITY * .5;
    public static final double CORAL_HOLD_VOLTAGE = RobotConfiguration.FEATURE_ALGAE_END_EFFECTOR ? 0 : -0.25;
    public static final double ALGAE_HOLD_VOLTAGE = -1.35;

    public static final double CORAL_COLLECTION_DELAY = RobotConfiguration.FEATURE_BEAN_BAKE ? 0.02 : 0.05;
    public static final double ALGAE_COLLECTION_DELAY = 0.1;

    public static final double CORAL_CANRANGE_DISTANCE = 0.13;
    public static final double CORAL_CANRANGE_DISTANCE_TOLERANCE = 0.01;

    public static final double GEAR_REDUCTION;

    static {
        switch (Identity.robotID) {
            case BEEF, BEARRACUDA -> GEAR_REDUCTION = (1d / 9d);
            default -> GEAR_REDUCTION = (10d / 26d);
        }
    }

    public static double CORAL_COLLECTION_CURRENT_THRESHOLD;
    public static double ALGAE_COLLECTION_CURRENT_THRESHOLD = 60;

    static {
        CORAL_COLLECTION_CURRENT_THRESHOLD = switch (Identity.robotID) {
            case BEEF -> 15;
            case BEARRACUDA -> 16;
            default -> 20;
        };
    }

    // -- States --

    public enum GrabberState {
        DISABLED(MotionType.NONE, 0, false),
        CORAL_HOLDING(MotionType.VOLTAGE, CORAL_HOLD_VOLTAGE, false),
        CORAL_COLLECTING(MotionType.VELOCITY, CORAL_COLLECT_VELOCITY, false),
        ALGAE_HOLDING(MotionType.VOLTAGE, ALGAE_HOLD_VOLTAGE, false),
        ALGAE_COLLECTING(MotionType.VELOCITY, ALGAE_COLLECT_VELOCITY, false),
        AUTO_SCORING(MotionType.VELOCITY, SCORING_VELOCITY, false),
        MANUAL_SCORING(MotionType.VELOCITY, SCORING_VELOCITY, true),
        L1_SCORING(MotionType.VELOCITY, -L1_SCORING_VELOCITY, true);

        public final MotionType type;
        public final double value;
        public final boolean usingSupplier;

        GrabberState(MotionType type, double value, boolean usingSupplier) {
            this.type = type;
            this.value = value;
            this.usingSupplier = usingSupplier;
        }

        public enum MotionType {
            POSITION, VELOCITY, VOLTAGE, NONE
        }
    }

    // -- Configuration --

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        // SysId'd 02/11
        .withSlot0(new Slot0Configs()
                       .withKP(1.048663)
                       .withKS(0.05988)
                       .withKV(0.37575)
                       .withKA(0.0053009))
        .withMotorOutput(new MotorOutputConfigs()
                             .withNeutralMode(NeutralModeValue.Brake)
                             .withInverted(InvertedValue.CounterClockwise_Positive))
        .withMotionMagic(new MotionMagicConfigs()
                             .withMotionMagicCruiseVelocity(40)
                             .withMotionMagicAcceleration(100)
                             .withMotionMagicJerk(5000))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / GEAR_REDUCTION))
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));

    public static final CANrangeConfiguration canRangeConfig = new CANrangeConfiguration()
        .withProximityParams(new ProximityParamsConfigs()
                             .withProximityThreshold(0.17)     // Meters
                             .withProximityHysteresis(0.003)) // Meters
        .withFovParams(new FovParamsConfigs()
                             .withFOVRangeX(6.75)   // Degrees
                             .withFOVRangeY(6.75)); // Degrees
}
