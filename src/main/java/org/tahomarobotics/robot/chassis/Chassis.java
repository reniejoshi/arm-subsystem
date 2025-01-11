package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.chassis.commands.AlignSwerveCommand;
import org.tahomarobotics.robot.util.CalibrationData;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Chassis extends SubsystemIF {
    private static final Chassis INSTANCE = new Chassis();

    // Swerve Modules

    @Logged(name = "modules/frontLeft")
    private final SwerveModule frontLeft;
    @Logged(name = "modules/frontRight")
    private final SwerveModule frontRight;
    @Logged(name = "modules/backLeft")
    private final SwerveModule backLeft;
    @Logged(name = "modules/backRight")
    private final SwerveModule backRight;

    private final List<SwerveModule> modules;

    // Gyro

    private final Pigeon2 pigeon = new Pigeon2(RobotMap.PIGEON, RobotConfiguration.CANBUS_NAME);

    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    @Logged
    public record ValidYaw(Rotation2d yaw, boolean valid) {}

    // State

    @Logged
    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    @Logged
    private Rotation2d heading = new Rotation2d();
    private SwerveModulePosition[] lastModulePosition;

    @Logged
    private final CalibrationData<Double[]> swerveCalibration;

    @Logged
    private final boolean isFieldCentric = true;
    private final Field2d fieldPose = new Field2d();

    // Models

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveDriveLimiter accelerationLimiter;

    private final Thread odometryThread;

    // Simulation

    @Logged
    private boolean isUsingHighFidelitySimulation = false;

    // Initialization

    private Chassis() {
        // Read calibration from rio
        swerveCalibration = new CalibrationData<>("SwerveCalibration", new Double[]{0d, 0d, 0d, 0d});

        // Use calibration to make modules
        Double[] angularOffsets = swerveCalibration.get();
        frontLeft = new SwerveModule(RobotMap.FRONT_LEFT_MOD, angularOffsets[0]);
        frontRight = new SwerveModule(RobotMap.FRONT_RIGHT_MOD, angularOffsets[1]);
        backLeft = new SwerveModule(RobotMap.BACK_LEFT_MOD, angularOffsets[2]);
        backRight = new SwerveModule(RobotMap.BACK_RIGHT_MOD, angularOffsets[3]);
        modules = List.of(frontLeft, frontRight, backLeft, backRight);

        kinematics = new SwerveDriveKinematics(
            modules.stream()
                   .map(SwerveModule::getTranslationOffset)
                   .toArray(Translation2d[]::new)
        );

        lastModulePosition = getSwerveModulePositions();

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(),
            getSwerveModulePositions(),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0))
        );

        accelerationLimiter = new SwerveDriveLimiter(getSwerveModuleStates(), ChassisConstants.ACCELERATION_LIMIT);

        odometryThread = new Thread(this::odometryThread);
        odometryThread.start();
    }

    public static Chassis getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        SmartDashboard.putData("AlignSwerve", new AlignSwerveCommand());
        zeroHeading();

        var gyro = getYaw().yaw;
        var modules = getSwerveModulePositions();
        synchronized (poseEstimator) {
            //changed SwerveDriveWheelPositions to passing in an array of the module positions
            //because SwerveDriveWheelPositions was just deleted in 2025.1.1 beta
            //if that broke something blame me :(
            poseEstimator.resetPosition(gyro, modules, new Pose2d());
        }

        return this;
    }

    // Calibration

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

    // Getters

    @Logged(name = "pose2d")
    public Pose2d getPose() {
        synchronized (poseEstimator) {
            return poseEstimator.getEstimatedPosition();
        }
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    @Logged(name = "states")
    public SwerveModuleState[] getSwerveModuleStates() {
        return modules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
    }

    @Logged(name = "chassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    @Logged(name = "yaw")
    public ValidYaw getYaw() {
        boolean valid = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        return new ValidYaw(Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawVelocity)), valid);
    }

    // Setters

    public void drive(ChassisSpeeds velocity, boolean isFieldCentric) {
        if (isFieldCentric) {
            velocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, getPose().getRotation());
        }
        targetSpeeds = velocity;
    }

    public void drive(ChassisSpeeds velocity) {
        if (!isFieldCentric && DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red) {
            velocity = new ChassisSpeeds(-velocity.vxMetersPerSecond, -velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond);
        }

        drive(velocity, isFieldCentric);
    }

    private void setSwerveStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules.get(i).setDesiredState(states[i]);
        }
    }

    public void zeroHeading() {
        pigeon.setYaw(0);
    }

    // Odometry

    private void odometryThread() {
        Threads.setCurrentThreadPriority(true, 1);

        // Get signals array
        List<BaseStatusSignal> signalList = new ArrayList<>(getStatusSignals());
        for (var module : this.modules) {
            signalList.addAll(module.getStatusSignals());
        }

        BaseStatusSignal[] signals = signalList.toArray(BaseStatusSignal[]::new);

        while (true) {
            // Wait for all signals to arrive
            BaseStatusSignal.waitForAll(4 / RobotConfiguration.ODOMETRY_UPDATE_FREQUENCY, signals);
            updatePosition();
        }
    }

    private SwerveModulePosition[] calculateModuleDeltas(SwerveModulePosition[] last, SwerveModulePosition[] current) {
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[current.length];
        for (int i = 0; i < current.length; i++) {
            moduleDeltas[i] = new SwerveModulePosition(current[i].distanceMeters - last[i].distanceMeters, current[i].angle);
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

            if (validYaw.valid()) { // If pigeon yaw is valid, accept it as the real value
                heading = validYaw.yaw();
            } else { // Else, calculate yaw from odometry by getting position deltas
                SwerveModulePosition[] deltas = calculateModuleDeltas(lastModulePosition, positions);
                Twist2d twist = kinematics.toTwist2d(deltas);
                heading = heading.plus(new Rotation2d(twist.dtheta));
            }
        }

        lastModulePosition = Arrays.copyOf(positions, positions.length);
        synchronized (poseEstimator) {
            poseEstimator.update(heading, positions);
        }
    }

    // Periodic

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
    }

    // Simulation

    private double lastUpdate = Timer.getFPGATimestamp();
    private double simHeading = 0;

    /** Enables high fidelity simulation. */
    public void useHighFidelitySimulation() {
        isUsingHighFidelitySimulation = true;
    }

    /** Enables low fidelity simulation. */
    public void useLowFidelitySimulation() {
        isUsingHighFidelitySimulation = false;
    }

    @Override
    public void onSimulationInit() {
        getStatusSignals().forEach(s -> s.setUpdateFrequency(1000));

        modules.forEach(SwerveModule::simulationInit);
    }

    @Override
    public void simulationPeriodic() {
        modules.forEach(m -> m.simulationPeriodic(isUsingHighFidelitySimulation));

        double currentTime = Timer.getFPGATimestamp();
        double dT = Timer.getFPGATimestamp() - lastUpdate;
        lastUpdate = currentTime;

        if (isUsingHighFidelitySimulation) {
            pigeon.getSimState().addYaw(getChassisSpeeds().omegaRadiansPerSecond * dT);
        } else {
            double dTheta = Units.radiansToDegrees(getChassisSpeeds().omegaRadiansPerSecond) * dT;
            simHeading += dTheta;
            pigeon.getSimState().setRawYaw(simHeading);
            pigeon.getSimState().setAngularVelocityZ(dTheta);
        }
    }

    // Status Signals

    private List<BaseStatusSignal> getStatusSignals() {
        return List.of(yaw, yawVelocity);
    }
}
