package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.auto.AutonomousConstants;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.persistent.CalibrationData;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.swerve.SwerveDrivePoseEstimatorDiff;
import org.tahomarobotics.robot.vision.AprilTagCamera;
import org.tinylog.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class Chassis extends SubsystemIF {
    private static final Chassis INSTANCE = new Chassis();

    // Swerve Modules

    private final List<SwerveModule> modules;

    // Gyro

    private final Pigeon2 pigeon = new Pigeon2(RobotMap.PIGEON, RobotConfiguration.CANBUS_NAME);

    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    private final LoggedStatusSignal[] statusSignals = new LoggedStatusSignal[]{
        new LoggedStatusSignal("Yaw", yaw),
        new LoggedStatusSignal("Yaw Velocity", yawVelocity)
    };

    @AutoLogOutput(key = "Chassis/Using Heading Fallback?")
    private boolean isUsingHeadingFallback = false;

    @AutoLogOutput(key = "Chassis/Auto Aligning?")
    private boolean autoAligning = false;
    @AutoLogOutput(key = "Chassis/Auto Alignment Offset")
    private double autoAligningOffset = 1;

    // State

    @AutoLogOutput(key = "Chassis/Target Speeds")
    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    private Rotation2d heading = new Rotation2d();
    private SwerveModulePosition[] lastModulePosition;

    @AutoLogOutput(key = "Chassis/Is Field Centric?")
    private final boolean isFieldCentric = true;
    private final CalibrationData<Double[]> swerveCalibration;
    private final Field2d fieldPose = new Field2d();

    // Models

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimatorDiff poseEstimator;

    private final SwerveDriveLimiter accelerationLimiter;

    // Simulation

    // -- Initialization --

    private Chassis() {
        // Read calibration from rio
        swerveCalibration = new CalibrationData<>("SwerveCalibration", new Double[]{0d, 0d, 0d, 0d});

        // Use calibration to make modules
        Double[] angularOffsets = swerveCalibration.get();
        modules = List.of(
            new SwerveModule(RobotMap.FRONT_LEFT_MOD, angularOffsets[0]),
            new SwerveModule(RobotMap.FRONT_RIGHT_MOD, angularOffsets[1]),
            new SwerveModule(RobotMap.BACK_LEFT_MOD, angularOffsets[2]),
            new SwerveModule(RobotMap.BACK_RIGHT_MOD, angularOffsets[3])
        );
        modules.forEach(AutoLogOutputManager::addObject);

        kinematics = new SwerveDriveKinematics(
            modules.stream()
                   .map(SwerveModule::getTranslationOffset)
                   .toArray(Translation2d[]::new)
        );

        lastModulePosition = getSwerveModulePositions();

        poseEstimator = new SwerveDrivePoseEstimatorDiff(
            kinematics,
            new SwerveDriveOdometry(kinematics, new Rotation2d(), getSwerveModulePositions(), new Pose2d()),
            VecBuilder.fill(0.02, 0.02, 0.02) // TODO: Verify
        );

        accelerationLimiter = new SwerveDriveLimiter(getSwerveModuleStates());

        new Thread(this::odometryThread).start();
    }

    public static Chassis getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        SmartDashboard.putData("Align Swerve", ChassisCommands.createAlignSwerveCommand(this));
        pigeon.setYaw(0);

        var gyro = getYaw().yaw;
        var modules = getSwerveModulePositions();
        synchronized (poseEstimator) {
            poseEstimator.resetPosition(gyro, modules, new Pose2d());
        }

        return this;
    }

    // -- Calibration --

    public void initializeCalibration() {
        modules.forEach(SwerveModule::initializeCalibration);
    }

    public void finalizeCalibration() {
        swerveCalibration.set(
            modules.stream()
                   .map(SwerveModule::finalizeCalibration)
                   .toArray(Double[]::new)
        );
    }

    public void cancelCalibration() {
        modules.forEach(SwerveModule::cancelCalibration);
    }

    // -- Getters --

    public Field2d getField() {
        return fieldPose;
    }

    @AutoLogOutput(key = "Chassis/Integrated Pose")
    public Pose2d getPose() {
        synchronized (poseEstimator) {
            return poseEstimator.getEstimatedPosition();
        }
    }

    /**
     * Returns the estimated pose at the supplied timestamp.
     *
     * @param timestamp Timestamp to sample
     *
     * @return The sampled pose
     */
    public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
        synchronized (poseEstimator) {
            return poseEstimator.sampleAt(timestamp);
        }
    }

    @AutoLogOutput(key = "Chassis/Odometry Pose")
    public Pose2d getRawPose() {
        synchronized (poseEstimator) {
            return poseEstimator.getRawPose();
        }
    }

    public List<SwerveModule> getModules() { return modules; }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    @AutoLogOutput(key = "Chassis/Module States")
    public SwerveModuleState[] getSwerveModuleStates() {
        return modules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    @AutoLogOutput(key = "Chassis/Chassis Speeds")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    @AutoLogOutput(key = "Chassis/Yaw")
    public ValidYaw getYaw() {
        boolean valid = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        return new ValidYaw(
            Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawVelocity)), valid);
    }

    @AutoLogOutput(key = "Chassis/Heading")
    public Rotation2d getHeading() {
        return heading;
    }

    // -- Setters --

    public void drive(ChassisSpeeds velocity, boolean isFieldCentric) {
        if (isFieldCentric) {
            velocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getPose().getRotation());
        }
        targetSpeeds = velocity;
    }

    public void drive(ChassisSpeeds velocity) {
        if (!isFieldCentric && DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            velocity = new ChassisSpeeds(
                -velocity.vxMetersPerSecond, -velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond);
        }

        drive(velocity, isFieldCentric);
    }

    public void autoDrive(ChassisSpeeds velocity) {
        drive(velocity, false);
    }

    private void setSwerveStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules.get(i).setDesiredState(states[i]);
        }
    }

    public void resetOdometry(Pose2d pose) {
        var modules = getSwerveModulePositions();
        synchronized (poseEstimator) {
            poseEstimator.resetPosition(getHeading(), modules, pose);
        }
        Logger.warn("Reset Pose: {}", pose);
    }

    public void orientToZeroHeading() {
        Rotation2d heading = new Rotation2d(
            DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue ? 0.0 : Math.PI);
        resetOdometry(new Pose2d(getPose().getTranslation(), heading));
    }

    public void setAutoAligning(boolean is) {
        autoAligning = is;
    }

    public boolean isAutoAligning() {
        return autoAligning;
    }

    public void setAutoAligningOffset(double offset) {
        autoAligningOffset = offset;

        AutonomousConstants.computePolePositions(Units.inchesToMeters(offset));
    }

    public void incrementAutoAligningOffset(double increment) {
        setAutoAligningOffset(autoAligningOffset + increment);
    }

    // -- Odometry --

    private void odometryThread() {
        Threads.setCurrentThreadPriority(true, 1);

        // Get signals array
        List<LoggedStatusSignal> statusSignals_ = new ArrayList<>(List.of(this.statusSignals));
        for (var module : this.modules) {
            statusSignals_.addAll(List.of(module.getStatusSignals()));
        }
        LoggedStatusSignal[] statusSignals = statusSignals_.toArray(LoggedStatusSignal[]::new);

        while (true) {
            // Wait for all signals to arrive
            LoggedStatusSignal.waitForAll(4 / RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY, statusSignals);
            updatePosition();
        }
    }

    private SwerveModulePosition[] calculateModuleDeltas(SwerveModulePosition[] last, SwerveModulePosition[] current) {
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[current.length];
        for (int i = 0; i < current.length; i++) {
            moduleDeltas[i] = new SwerveModulePosition(
                current[i].distanceMeters - last[i].distanceMeters, current[i].angle);
        }

        return moduleDeltas;
    }

    private void updatePosition() {
        SwerveModulePosition[] positions;

        synchronized (modules) {
            positions = getSwerveModulePositions();
        }

        synchronized (pigeon) {
            var validYaw = getYaw();
            isUsingHeadingFallback = !validYaw.valid();

            if (validYaw.valid()) { // If pigeon yaw is valid, accept it as the real value
                heading = validYaw.yaw();
            } else { // Else, calculate yaw from odometry by getting position deltas
                SwerveModulePosition[] deltas = calculateModuleDeltas(lastModulePosition, positions);
                Twist2d twist = kinematics.toTwist2d(deltas);
                heading = heading.plus(new Rotation2d(twist.dtheta));
            }

            heading = new Rotation2d(MathUtil.angleModulus(heading.getRadians()));
        }

        lastModulePosition = Arrays.copyOf(positions, positions.length);
        synchronized (poseEstimator) {
            poseEstimator.update(heading, positions);
        }
    }

    public void processVisionUpdate(AprilTagCamera.EstimatedRobotPose estimatedRobotPose) {
        synchronized (poseEstimator) {
            poseEstimator.addVisionMeasurement(
                estimatedRobotPose.pose(),
                estimatedRobotPose.timestamp(),
                estimatedRobotPose.stdDevs()
            );
        }
    }

    // -- Periodic --

    @Override
    public void periodic() {
        modules.forEach(SwerveModule::periodic);

        Pose2d pose = getPose();

        fieldPose.setRobotPose(pose);
        SmartDashboard.putData(fieldPose);

        if (RobotState.isEnabled()) {
            var swerveModuleStates = kinematics.toSwerveModuleStates(targetSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.MAX_VELOCITY);
            swerveModuleStates = accelerationLimiter.calculate(swerveModuleStates);

            setSwerveStates(swerveModuleStates);
        }

        LoggedStatusSignal.log("Chassis/", statusSignals);
    }

    // -- Simulation --

    private double lastUpdate = Timer.getFPGATimestamp();

    @Override
    public void onSimulationInit() {
        modules.forEach(SwerveModule::simulationInit);
    }

    @Override
    public void simulationPeriodic() {
        modules.forEach(SwerveModule::simulationPeriodic);

        double currentTime = Timer.getFPGATimestamp();
        double dT = Timer.getFPGATimestamp() - lastUpdate;
        lastUpdate = currentTime;

        double omegaDegreesPerSecond = Units.radiansToDegrees(getChassisSpeeds().omegaRadiansPerSecond * dT);
        pigeon.getSimState().addYaw(omegaDegreesPerSecond);
    }

    // -- Status Signals --

    @AutoLogOutput(key = "Chassis/Total Drive Current")
    public double getTotalDriveCurrent() {
        double totalCurrent = 0;
        for (double current : modules.stream().map(SwerveModule::getDriveCurrent).toList()) {
            totalCurrent += current;
        }
        return totalCurrent;
    }

    // -- Records --

    public record ValidYaw(Rotation2d yaw, boolean valid) {}
}
