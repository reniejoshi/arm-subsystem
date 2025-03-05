package org.tahomarobotics.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tinylog.Logger;
import org.tinylog.TaggedLogger;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import static org.tahomarobotics.robot.vision.VisionConstants.*;

// TODO: Put a ChArUco board in the repository.

/**
 * A PhotonVision camera configured to handle AprilTags and
 * estimate the robot's position with CasADi. A benefit to CasADi over MultiTag and
 * unconstrained SolvePNP is that when it's wrong, it's <strong>really</strong> wrong,
 * making it easier to filter out.
 *
 * <p><strong>PhotonVision Configuration</strong><p>
 * <p>
 * The PhotonVision camera instance must correspond the supplied naming in the configuration
 * and be as exact to the transform as possible. Furthermore, the camera should be properly
 * calibrated (either importing a prior calibration from the same camera model or calibrating
 * using a ChArUco board) and be using an 3D AprilTag pipeline (All estimation happens on the RoboRIO and
 * only relies on targets' corner positions).
 *
 * <p><strong>AprilTag Layout</strong><p>
 * <p>
 * Constrained estimation with CasADi requires pitch, height, and yaw (toggleable) to be fixed and any errors
 * with them will result in extremely inaccurate estimations. As such, the
 * AprilTag layout in {@link VisionConstants} and the ones uploaded to the PhotonVision instances should correspond
 * to the physical placement as accurately as possible (which will most likely not be perfect). Prior to matches,
 * AprilTag placement should be evaluated using <a href="https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/wpical/index.html"
 * >WPICal</a> with the resulting layout placed at <code>src/main/deploy</code> and uploaded to PhotonVision.
 *
 * <p><strong>Tuning</strong><p>
 * <p>
 * There are a lot of values that can be tuned to increase accuracy of estimations, but most often, scaling standard
 * deviations will be enough to get an accurate estimate. To do this,
 * <ol>
 *      <li>Place the robot a known distance away from an individual AprilTag (you may isolate a single target in code)</li>
 *      <li>Record the distance to the tag from the center of the robot and the error from the estimation</li>
 *      <li>Repeat for several distances</li>
 *      <li>Curve fit the data to get a scaling function for our standard deviations</li>
 *      <li>If necessary, repeat for scenarios with multiple AprilTags or use a scaled version of the single-tag function,
 *    but more likely than not estimations with multiple AprilTags will be extremely accurate
 *    (if they are not then standard deviations can't fix them).</li>
 * </ol>
 */
public class AprilTagCamera implements AutoCloseable {
    private final TaggedLogger logger;

    // Camera

    private final String name;
    protected final CameraConfiguration configuration;

    protected final PhotonCamera camera;
    protected final PhotonCameraSim sim;

    private final Matrix<N3, N3> cameraMatrix;
    private final Matrix<N8, N1> distortionCoefficients;

    private final Consumer<EstimatedRobotPose> estimationCallback;
    private final PhotonPoseEstimator isolationPoseEstimator;

    @SuppressWarnings("FieldCanBeLocal")
    private final Notifier notifier;

    // Diagnostics

    // Vision measurements that have been adjusted and are passed to the callback.
    @AutoLogOutput(key = "Vision/{name}/Multi-Tag Pose")
    private Pose2d multiTagPose = new Pose2d();
    @AutoLogOutput(key = "Vision/{name}/Single-Tag Pose")
    private Pose2d singleTagPose = new Pose2d();

    @AutoLogOutput(key = "Vision/{name}/AprilTags")
    private Pose3d[] aprilTags = new Pose3d[0];
    @AutoLogOutput(key = "Vision/{name}/AprilTag IDs")
    private int[] aprilTagIDs = new int[0];

    @AutoLogOutput(key = "Vision/{name}/Isolated Tag")
    private int isolationTarget = -1;

    @AutoLogOutput(key = "Vision/{name}/Failed Updates")
    private int failedUpdates = 0;
    @AutoLogOutput(key = "Vision/{name}/Single-Tag Updates")
    private int singleTagUpdates = 0;
    @AutoLogOutput(key = "Vision/{name}/Multi-Tag Updates")
    private int multiTagUpdates = 0;
    @AutoLogOutput(key = "Vision/{name}/Estimation Time")
    private double estimationTime = 0;
    @AutoLogOutput(key = "Vision/{name}/Camera-to-Robot Latency")
    private double photonvisionLatency = 0;
    @AutoLogOutput(key = "Vision/{name}/Processing Time")
    private double processingTime = 0;

    // Initialization

    /**
     * Creates a new {@link AprilTagCamera} with the supplied configuration and simulation properties.
     *
     * @param configuration Configuration about the camera (name, transform, etc.)
     * @param simProperties Simulation properties representing the real camera as closely as possible
     * @param callback      A callback to consume processed estimated robot poses
     */
    public AprilTagCamera(
        CameraConfiguration configuration,
        SimCameraProperties simProperties,
        Consumer<EstimatedRobotPose> callback
    ) {
        this.name = configuration.name();
        this.configuration = configuration;

        this.estimationCallback = callback;
        this.isolationPoseEstimator = new PhotonPoseEstimator(
            FIELD_LAYOUT, PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, configuration.transform());

        this.notifier = new Notifier(this::processUnreadVisionUpdates);

        logger = Logger.tag(configuration.name());

        camera = new PhotonCamera(configuration.name());
        sim = new PhotonCameraSim(camera, simProperties);

        if (camera.getCameraMatrix().isPresent()) {
            cameraMatrix = camera.getCameraMatrix().get();
        } else {
            logger.error("Could not retrieve camera matrix! Falling back to simulation properties.");
            cameraMatrix = simProperties.getIntrinsics(); // These should be identical.
        }

        if (camera.getDistCoeffs().isPresent()) {
            distortionCoefficients = camera.getDistCoeffs().get();
        } else {
            logger.error("Could not retrieve distortion coefficients! Falling back to simulation properties.");
            distortionCoefficients = simProperties.getDistCoeffs(); // ^
        }

        notifier.startPeriodic(Robot.defaultPeriodSecs);
    }

    // Processing

    /**
     * Processes all unread valid vision updates into estimated robot poses.
     */
    public void processUnreadVisionUpdates() {
        camera.getAllUnreadResults()
              .forEach(result -> {
                  var est = processUpdate(result);
                  est.ifPresent(estimationCallback);
              });
    }

    /**
     * Processes a {@link PhotonPipelineResult} into an estimated robot pose using a constrained SolvePNP
     * algorithm with CasADi.
     *
     * @param result The pipeline result
     *
     * @return An estimated pose, if possible
     */
    private Optional<EstimatedRobotPose> processUpdate(PhotonPipelineResult result) {
        Optional<Pose3d> chassisPose = Chassis.getInstance().getPoseAtTimestamp(result.getTimestampSeconds()).map(Pose3d::new);

        double timestamp = result.getTimestampSeconds();
        double now = Timer.getFPGATimestamp();

        // Publish AprilTag Positions
        chassisPose.ifPresent(pose -> {
            aprilTags = result.getTargets().stream()
                              .map(target -> pose.plus(configuration.transform()).plus(target.getBestCameraToTarget()))
                              .toArray(Pose3d[]::new);
            aprilTagIDs = result.getTargets().stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
        });

        // Pre-filtering

        List<PhotonTrackedTarget> targets = (result.getTargets().stream())
            .filter(t -> t.fiducialId <= FIELD_LAYOUT.getTags().size())
            .filter(t -> t.getPoseAmbiguity() <= AMBIGUITY_THRESHOLD)
            .toList();

        if (targets.isEmpty()) {
            return Optional.empty();
        }

        if (isolationTarget > 0) {
            isolationPoseEstimator.addHeadingData(
                result.getTimestampSeconds(), chassisPose.map(p -> p.getRotation().toRotation2d()).orElseGet(Chassis.getInstance()::getHeading));

            Optional<PhotonTrackedTarget> target_ = targets.stream().filter(t -> t.fiducialId == isolationTarget).findAny();
            if (target_.isEmpty()) { return Optional.empty(); }
            PhotonTrackedTarget target = target_.get();

            var est = isolationPoseEstimator.update(new PhotonPipelineResult(result.metadata, List.of(target), Optional.empty()));
            return est.map(e -> new EstimatedRobotPose(
                name, result.getTimestampSeconds(),
                EstimatedRobotPose.Type.ISOLATED_SINGLE_TAG,
                e.estimatedPose.toPose2d(),
                BASE_ISOLATED_SINGLE_TAG_STD_DEV,
                List.of(target)
            ));
        }

        // CasADi pose estimation
        // TODO: Implement once Constrained SolvePNP is released officially.
//        Pose3d casadiPose3d;
//        try {
//            casadiPose3d = estimateCasADiPose(
//                timestamp,
//                targets
//            );
//        } catch (Exception e) {
//            failedUpdates++;
//
//            logger.error(e.getMessage());
//            return Optional.empty();
//        }

        Pose3d robotPose;

        EstimatedRobotPose.Type type;
        Vector<N3> stdDevs;
        if (targets.size() > 1) {
            // Multi-Tag Pose Estimation

            Transform3d fieldToCamera, fieldToRobot;
            if (result.getMultiTagResult().isPresent()) {
                fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
            } else {
                logger.warn(
                    "Camera sees multiple tags but did not receive a Multi-Tag result! Ensure it is enabled in PhotonVision.");
                return Optional.empty();
            }

            fieldToRobot = fieldToCamera.plus(configuration.transform().inverse());
            robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            // Filter if the estimate was not in the field

            if (notInField(robotPose)) {
                failedUpdates++;
                return Optional.empty();
            }

            // Create an estimation result that is scaled based on the average distance to the tags

            double dist = (targets.stream())
                .filter(t -> result.getMultiTagResult().get().fiducialIDsUsed.contains((short) t.fiducialId))
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(-1);

            if (dist < 0) {
                failedUpdates++;
                return Optional.empty();
            }

            type = EstimatedRobotPose.Type.MULTI_TAG;
            stdDevs = (configuration)
                .stdDevScaling()
                .scaleStandardDeviations(BASE_MULTI_TAG_STD_DEV, dist, targets.size());

            multiTagPose = robotPose.toPose2d();
            multiTagUpdates++;
        } else {
            // Pose Estimation

            PhotonTrackedTarget tag = targets.get(0);
            robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                tag.getBestCameraToTarget(),
                FIELD_LAYOUT.getTagPose(tag.getFiducialId()).orElseThrow(),
                configuration.transform().inverse()
            );

            // Filter if the estimate was not in the field

            if (notInField(robotPose)) {
                failedUpdates++;
                return Optional.empty();
            }

            // Create an estimation result that is scaled based on the average distance to the tags

            double dist = tag.getBestCameraToTarget().getTranslation().getNorm();

            type = EstimatedRobotPose.Type.SINGLE_TAG;
            stdDevs = (configuration)
                .stdDevScaling()
                .scaleStandardDeviations(BASE_SINGLE_TAG_STD_DEV, dist, targets.size());

            singleTagPose = robotPose.toPose2d();
            singleTagUpdates++;
        }

        // Diagnostics

        estimationTime = Timer.getFPGATimestamp() - now;
        photonvisionLatency = result.metadata.getLatencyMillis() / 1000;
        processingTime = Timer.getFPGATimestamp() - timestamp;

        // ---

        EstimatedRobotPose estimation =
            new EstimatedRobotPose(
                this.configuration.name(), timestamp, type, robotPose.toPose2d(), stdDevs, result.targets);

        return Optional.of(estimation);
    }

    /**
     * Estimates the robot position at a timestamp using the AprilTags in view.
     *
     * @param timestamp The timestamp of the update
     * @param targets   The targets the camera saw at the update
     *
     * @return An estimated robot pose
     *
     * @throws Exception If the estimation fails for any reason, an exception will be thrown with a more descriptive
     *                   error message. These are unrecoverable for this update, but not for the program, so they are
     *                   checked exceptions.
     */
    private Pose3d estimateCasADiPose(double timestamp, List<PhotonTrackedTarget> targets) throws Exception {
        // Get the chassis position at the timestamp
        Pose2d pose = Chassis.getInstance()
                             .getPoseAtTimestamp(timestamp)
                             .orElseThrow(() -> new Exception(
                                 "Could not retrieve robot pose at " + timestamp + "!"));

        // TODO: Do not attempt estimation if the robot has an pitch or roll or is otherwise violating constraints.

        // Estimate the position of the robot
        PnpResult casADiResult = VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
            cameraMatrix,
            distortionCoefficients,
            targets,
            configuration.transform(),
            new Pose3d(pose),
            FIELD_LAYOUT,
            TargetModel.kAprilTag36h11,
            HEADING_FREE,
            pose.getRotation(),
            GYRO_ERROR_SCALING_FACTOR
        ).orElseGet(PnpResult::new);//.orElseThrow(() -> new Exception("Failed to estimate robot pose!"));

        // Add the resulting transform to the field origin
        return new Pose3d().plus(casADiResult.best);
    }

    // Helper Methods

    /**
     * Checks if the pose is within field bounds.
     *
     * @param pose The pose
     *
     * @return Whether the pose is in field bounds
     */
    private static boolean notInField(Pose3d pose) {
        return pose.getX() < ChassisConstants.HALF_TRACK_WIDTH || pose.getX() > FIELD_LAYOUT.getFieldLength() - ChassisConstants.HALF_TRACK_WIDTH ||
               pose.getY() < ChassisConstants.HALF_WHEELBASE || pose.getY() > FIELD_LAYOUT.getFieldWidth() - ChassisConstants.HALF_WHEELBASE;
    }

    // Getters

    /** Gets the name of this camera. */
    public String getName() {
        return configuration.name();
    }

    // Setters

    void isolate(int tag) {
        isolationTarget = tag;
    }

    void globalize() {
        isolationTarget = -1;
    }

    // Diagnostics

    /**
     * Whether the camera is currently connect        isolationPoseEstimator.setLastPose();
     * ed.
     */
    @AutoLogOutput(key = "Vision/{name}/Connected?")
    public boolean connected() {
        return camera.isConnected();
    }

    // Simulation

    /** Adds this camera to a vision simulation. */
    public void addCameraToSimulation(VisionSystemSim sim) {
        sim.addCamera(this.sim, configuration.transform());
    }


    // Processing Results

    /**
     * A timestamped estimated robot pose with associated trust values (standard deviations)
     * and the targets used for the estimation.
     */
    public record EstimatedRobotPose(String cameraName, double timestamp, AprilTagCamera.EstimatedRobotPose.Type type, Pose2d pose, Vector<N3> stdDevs,
                                     List<PhotonTrackedTarget> targets) {
        public enum Type {
            MULTI_TAG, SINGLE_TAG, ISOLATED_SINGLE_TAG
        }
    }

    // Auto Closeable

    @Override
    public void close() {
        sim.close();
        camera.close();
    }
}
