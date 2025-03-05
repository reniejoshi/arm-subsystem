package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.vision.VisionConstants;
import org.tinylog.Logger;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class AutonomousConstants {
    /** A horizontal shift on the robot's position relative to reef poles. */
    public static final double DEFAULT_REEF_HORIZONTAL_ALIGNMENT_FUDGE = Units.inchesToMeters(0);
    public static final double FUDGE_INCREMENT = 0.5; // Inches

    // Translational Constraints in Meters
    public static final TrapezoidProfile.Constraints TRANSLATION_ALIGNMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);
    public static final double TRANSLATION_ALIGNMENT_KP = 5, TRANSLATION_ALIGNMENT_KI = 0, TRANSLATION_ALIGNMENT_KD = 0.25;
    public static final double TRANSLATION_ALIGNMENT_TOLERANCE = 0.01;

    // Rotational Constraints in Radians
    public static final TrapezoidProfile.Constraints ROTATION_ALIGNMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI);
    public static final double ROTATION_ALIGNMENT_KP = 5, ROTATION_ALIGNMENT_KI = 0, ROTATION_ALIGNMENT_KD = 0.5;
    public static final double ROTATION_ALIGNMENT_TOLERANCE = Units.degreesToRadians(0.25);

    /** Distance between the centers of the reef poles on the same side of the reef. */
    private static final double DISTANCE_BETWEEN_REEF_POLES = Units.inchesToMeters(12.94);
    /** Perpendicular distance from the center of the reef to the center of the chassis once aligned. */
    private static final double SCORE_DISTANCE_FROM_CENTER = Units.inchesToMeters(32.75) + ChassisConstants.BUMPER_WIDTH / 2 + Units.inchesToMeters(1);
    private static final double APPROACH_DISTANCE_FROM_CENTER = SCORE_DISTANCE_FROM_CENTER + Units.inchesToMeters(12);
    public static final double APPROACH_DISTANCE_BLEND_FACTOR = Units.inchesToMeters(12);

    private static final Translation2d BLUE_REEF_CENTER = new Translation2d(
        Units.inchesToMeters(144 + 93.5 / 2 - 14),
        VisionConstants.FIELD_LAYOUT.getFieldWidth() / 2
    );
    private static final Translation2d RED_REEF_CENTER = new Translation2d(
        VisionConstants.FIELD_LAYOUT.getFieldLength() - Units.inchesToMeters(144 + 93.5 / 2 - 14),
        VisionConstants.FIELD_LAYOUT.getFieldWidth() / 2
    );

    private static List<Translation2d> RED_REEF_APPROACH_POLES, RED_REEF_SCORE_POLES, BLUE_REEF_APPROACH_POLES, BLUE_REEF_SCORE_POLES;

    static {
        computePolePositions(DEFAULT_REEF_HORIZONTAL_ALIGNMENT_FUDGE);
    }

    public static void computePolePositions(double fudge) {
        List<Translation2d> APPROACH_REEF_POLES =
            (IntStream.range(0, 12))
                .mapToObj(i -> new Translation2d(
                    APPROACH_DISTANCE_FROM_CENTER, (i % 2 == 0 ? -DISTANCE_BETWEEN_REEF_POLES : DISTANCE_BETWEEN_REEF_POLES) / 2 - fudge
                ).rotateBy(Rotation2d.fromDegrees(60).times(Math.floor((double) i / 2))))
                .toList();

        List<Translation2d> SCORE_REEF_POLES =
            (IntStream.range(0, 12))
                .mapToObj(i -> new Translation2d(
                    SCORE_DISTANCE_FROM_CENTER, (i % 2 == 0 ? -DISTANCE_BETWEEN_REEF_POLES : DISTANCE_BETWEEN_REEF_POLES) / 2 - fudge
                ).rotateBy(Rotation2d.fromDegrees(60).times(Math.floor((double) i / 2))))
                .toList();

        RED_REEF_APPROACH_POLES = APPROACH_REEF_POLES.stream().map(p -> p.plus(RED_REEF_CENTER)).collect(Collectors.toList());
        RED_REEF_SCORE_POLES = SCORE_REEF_POLES.stream().map(p -> p.plus(RED_REEF_CENTER)).collect(Collectors.toList());

        BLUE_REEF_APPROACH_POLES = APPROACH_REEF_POLES.stream().map(p -> p.plus(BLUE_REEF_CENTER)).collect(Collectors.toList());
        BLUE_REEF_SCORE_POLES = SCORE_REEF_POLES.stream().map(p -> p.plus(BLUE_REEF_CENTER)).collect(Collectors.toList());
        Collections.rotate(BLUE_REEF_APPROACH_POLES, 6);
        Collections.rotate(BLUE_REEF_SCORE_POLES, 6);
    }

    public static ReefPole getNearestReefPoleScorePosition(Translation2d currentTranslation) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElseGet(() -> {
            Logger.error("Alliance not specified! Defaulting to Blue...");
            return DriverStation.Alliance.Blue;
        });

        var approachPoles = alliance == DriverStation.Alliance.Blue ? BLUE_REEF_APPROACH_POLES : RED_REEF_APPROACH_POLES;
        var scorePoles = alliance == DriverStation.Alliance.Blue ? BLUE_REEF_SCORE_POLES : RED_REEF_SCORE_POLES;

        Translation2d approach = currentTranslation.nearest(approachPoles);
        int index = approachPoles.indexOf(approach);
        Translation2d score = scorePoles.get(index);

        Rotation2d angle = Rotation2d.fromDegrees(60).times(Math.floor((double) index / 2))
                                     .plus(alliance == DriverStation.Alliance.Blue ? Rotation2d.k180deg : Rotation2d.kZero);

        return new ReefPole(index, getReefAprilTagForPole(index), new Pose2d(approach, angle), new Pose2d(score, angle));
    }

    public static int getReefAprilTagForPole(int poleIndex) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElseGet(() -> {
            Logger.error("Alliance not specified! Defaulting to Blue...");
            return DriverStation.Alliance.Blue;
        });

        int side = poleIndex / 2;
        if (alliance == DriverStation.Alliance.Blue) {
            return side < 2 ? 18 - side : 22 - (side - 2);
        } else {
            return side < 5 ? 7 + side : 6;
        }
    }
    
    public record ReefPole(int index, int aprilTagId, Pose2d approachPose, Pose2d scorePose) {}
}