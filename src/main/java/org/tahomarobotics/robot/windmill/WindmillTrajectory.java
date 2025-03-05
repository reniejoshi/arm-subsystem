package org.tahomarobotics.robot.windmill;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState;
import org.tinylog.Logger;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

public class WindmillTrajectory {
    public final static File BEEF_SAVE_DIR = new File(RobotConfiguration.DEPLOY_DIR, "beef");

    private final Trajectory backingTrajectory;
    private final List<Orientation> orientations;

    private WindmillState previousState;

    public WindmillTrajectory(
        TrajectoryConfig config, List<Orientation> orientations, List<Translation2d> points, Translation2d startTangent,
        Translation2d endTangent
    ) {
        assert !points.isEmpty();

        int last = points.size() - 1;
        var start = new Spline.ControlVector(
            new double[]{points.get(0).getX(), startTangent.getX()},
            new double[]{points.get(0).getY(), startTangent.getY()}
        );
        var end = new Spline.ControlVector(
            new double[]{points.get(last).getX(), endTangent.getX()},
            new double[]{points.get(last).getY(), endTangent.getY()}
        );
        var midpoints = new ArrayList<Translation2d>();

        for (int i = 1; i < last; i++) {
            midpoints.add(new Translation2d(points.get(i).getX(), points.get(i).getY()));
        }

        this.orientations = orientations;
        backingTrajectory = TrajectoryGenerator.generateTrajectory(
            start, midpoints, end, config
        );
    }

    // Getters

    public WindmillState sampleWindmillState(double t) throws WindmillKinematics.KinematicsException {
        Trajectory.State state = backingTrajectory.sample(t);
        boolean isUp = false; // The default value will never be used.
        for (int i = 1; i <= this.orientations.size(); i++) {
            if (i == orientations.size() || orientations.get(i).time > t) {
                isUp = orientations.get(i - 1).isUp;
                break;
            }
        }
        previousState = WindmillKinematics.inverseKinematics(t, state.poseMeters.getTranslation(), previousState, isUp);
        return previousState;
    }

    public Trajectory.State sampleBackingTrajectory(double t) {
        return backingTrajectory.sample(t);
    }

    public double getDuration() {
        return backingTrajectory.getTotalTimeSeconds();
    }

    public Trajectory getBackingTrajectory() {
        return backingTrajectory;
    }

    // Generation

    public static Optional<WindmillTrajectory> loadFromNetworkTables() {
        return loadFromJSON(SmartDashboard.getString("BEEF/Serialized", null));
    }

    public static Optional<WindmillTrajectory> loadFromFromTo(Pair<TrajectoryState, TrajectoryState> fromTo) {
        Logger.info("Loading trajectory {}", fromTo);

        if (fromTo == null) { return loadFromNetworkTables(); }
        File file = new File(BEEF_SAVE_DIR, getFileNameWithExtension(fromTo) + ".traj");

        try {
            var trajectory_ = loadFromJSON(Files.readString(file.toPath()));

            if (trajectory_.isPresent()) {
                var trajectory = trajectory_.get();
                for (double t = 0; t <= trajectory.getDuration(); t += Robot.defaultPeriodSecs) {
                    try {
                        trajectory.sampleWindmillState(t);
                    } catch (WindmillKinematics.KinematicsException e) {
                        Logger.error("Failed to validate trajectory at {} seconds.", t);

                        return Optional.empty();
                    }
                }
            }

            return trajectory_;
        } catch (IOException e) {
            return Optional.empty();
        }
    }

    /**
     * Loads a trajectory from a trajectory JSON string.
     *
     * @param json The JSON string
     *
     * @return A trajectory if valid
     */
    public static Optional<WindmillTrajectory> loadFromJSON(String json) {
        try {
            ObjectMapper mapper = new ObjectMapper();
            JsonNode node = mapper.readTree(json);

            TrajectoryConfig config = new TrajectoryConfig(
                node.path("config").get("max_velocity").asDouble(),
                node.path("config").get("max_acceleration").asDouble()
            );

            List<Orientation> orientations = new ArrayList<>();
            for (JsonNode o : node.path("orientations")) {
                double time = o.get(0).asDouble();
                boolean orientation = Objects.equals(o.get(1).asText(), "UP");

                orientations.add(new Orientation(time, orientation));
            }

            List<Translation2d> points = new ArrayList<>();
            for (JsonNode o : node.path("points")) {
                points.add(new Translation2d(o.get(0).asDouble(), o.get(1).asDouble()));
            }

            Translation2d startTangent = new Translation2d(
                node.path("start_tangent").get(0).asDouble(), node.path("start_tangent").get(1).asDouble());
            Translation2d endTangent = new Translation2d(
                node.path("end_tangent").get(0).asDouble(), node.path("end_tangent").get(1).asDouble());

            return Optional.of(new WindmillTrajectory(config, orientations, points, startTangent, endTangent));
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    public static String getFileNameWithExtension(Pair<TrajectoryState, TrajectoryState> fromTo) {
        if (fromTo.getFirst() == null || fromTo.getSecond() == null) {
            return "<unnamed>";
        }

        return fromTo.getFirst() + "_TO_" + fromTo.getSecond();
    }

    public record Orientation(double time, Boolean isUp) {}
}
