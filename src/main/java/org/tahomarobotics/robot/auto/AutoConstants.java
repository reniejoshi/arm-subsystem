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

public class AutoConstants {
    /** A horizontal shift on the robot's position relative to reef poles. */
    public static final double REEF_HORIZONTAL_ALIGNMENT_FUDGE = Units.inchesToMeters(4);

    // Translational Constraints in Meters
    public static final TrapezoidProfile.Constraints TRANSLATION_ALIGNMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 4.5);
    public static final double TRANSLATION_ALIGNMENT_KP = 5, TRANSLATION_ALIGNMENT_KI = 0, TRANSLATION_ALIGNMENT_KD = 0;
    public static final double TRANSLATION_ALIGNMENT_TOLERANCE = 0.01;

    // Rotational Constraints in Radians
    public static final TrapezoidProfile.Constraints ROTATION_ALIGNMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI);
    public static final double ROTATION_ALIGNMENT_KP = 5, ROTATION_ALIGNMENT_KI = 0, ROTATION_ALIGNMENT_KD = 0;
    public static final double ROTATION_ALIGNMENT_TOLERANCE = Units.degreesToRadians(0.25);

    /** Distance between the centers of the reef poles on the same side of the reef. */
    private static final double DISTANCE_BETWEEN_REEF_POLES = Units.inchesToMeters(12.94);
    /** Perpendicular distance from the center of the reef to the center of the chassis once aligned. */
    private static final double DISTANCE_FROM_CENTER = Units.inchesToMeters(32.75) + ChassisConstants.BUMPER_WIDTH / 2;

    private static final Translation2d BLUE_REEF_CENTER = new Translation2d(
        Units.inchesToMeters(144 + 93.5 / 2 - 14),
        VisionConstants.FIELD_LAYOUT.getFieldWidth() / 2
    );
    private static final Translation2d RED_REEF_CENTER = new Translation2d(
        VisionConstants.FIELD_LAYOUT.getFieldLength() - Units.inchesToMeters(144 + 93.5 / 2 - 14),
        VisionConstants.FIELD_LAYOUT.getFieldWidth() / 2
    );

    private static final List<Translation2d> RED_REEF_POLES, BLUE_REEF_POLES;

    static {
        List<Translation2d> REEF_POLES =
            (IntStream.range(0, 12))
                .mapToObj(i -> new Translation2d(
                    DISTANCE_FROM_CENTER, (i % 2 == 0 ? -DISTANCE_BETWEEN_REEF_POLES : DISTANCE_BETWEEN_REEF_POLES) / 2 - REEF_HORIZONTAL_ALIGNMENT_FUDGE
                ).rotateBy(Rotation2d.fromDegrees(60).times(Math.floor((double) i / 2))))
                .toList();

        RED_REEF_POLES = REEF_POLES.stream().map(p -> p.plus(RED_REEF_CENTER)).collect(Collectors.toList());
        BLUE_REEF_POLES = REEF_POLES.stream().map(p -> p.plus(BLUE_REEF_CENTER)).collect(Collectors.toList());
        Collections.rotate(BLUE_REEF_POLES, 6);
    }

    public static Pose2d getNearestReefPoleScorePosition(Translation2d currentTranslation) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElseGet(() -> {
            Logger.error("Alliance not specified! Defaulting to Blue...");
            return DriverStation.Alliance.Blue;
        });

        var poles = alliance == DriverStation.Alliance.Blue ? BLUE_REEF_POLES : RED_REEF_POLES;
        Translation2d target = currentTranslation.nearest(poles);
        int index = poles.indexOf(target);

        return new Pose2d(target, Rotation2d.fromDegrees(60).times(Math.floor((double) index / 2)).plus(Rotation2d.k180deg));
    }

    public static final String DEFAULT_AUTO_NAME = "No Operation";
}