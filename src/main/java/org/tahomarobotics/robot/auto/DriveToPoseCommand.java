package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tinylog.Logger;

import static org.tahomarobotics.robot.auto.AutoConstants.*;

// Adapted from another team, not sure who though...
public class DriveToPoseCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();

    private final ProfiledPIDController translationController, rotationController;

    private Translation2d lastSetpoint;
    private final Pose2d goalPose;

    public DriveToPoseCommand(Pose2d goalPose) {
        this.goalPose = goalPose;

        translationController = new ProfiledPIDController(
            TRANSLATION_ALIGNMENT_KP, TRANSLATION_ALIGNMENT_KI, TRANSLATION_ALIGNMENT_KD,
            TRANSLATION_ALIGNMENT_CONSTRAINTS
        );
        translationController.setTolerance(TRANSLATION_ALIGNMENT_TOLERANCE);

        rotationController = new ProfiledPIDController(
            ROTATION_ALIGNMENT_KP, ROTATION_ALIGNMENT_KI, ROTATION_ALIGNMENT_KD,
            ROTATION_ALIGNMENT_CONSTRAINTS
        );
        rotationController.setTolerance(ROTATION_ALIGNMENT_TOLERANCE);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = chassis.getPose();
        ChassisSpeeds currentVelocity = getFieldRelativeChassisSpeeds(chassis.getChassisSpeeds(), currentPose);

        Logger.info("Driving to {} from {}", goalPose, currentPose);

        translationController.reset(
            currentPose.getTranslation().getDistance(goalPose.getTranslation()),
            Math.min(
                0.0,
                new Translation2d(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond)
                    .rotateBy(
                        goalPose
                            .getTranslation()
                            .minus(currentPose.getTranslation())
                            .getAngle()
                            .unaryMinus())
                    .getX() * -1
            )
        );

        rotationController.reset(
            currentPose.getRotation().getRadians(),
            currentVelocity.omegaRadiansPerSecond
        );

        lastSetpoint = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        Pose2d currentPose = chassis.getPose();
        double distanceToGoalPose = currentPose.getTranslation().getDistance(goalPose.getTranslation());

        double ffScaler = MathUtil.clamp((distanceToGoalPose - 0.2) / (0.8 - 0.2), 0.0, 1.0);

        translationController.reset(
            lastSetpoint.getDistance(goalPose.getTranslation()),
            translationController.getSetpoint().velocity
        );

        double driveVelocityScalar = translationController.getSetpoint().velocity * ffScaler
                                     + translationController.calculate(distanceToGoalPose, 0.0);

        if (distanceToGoalPose < translationController.getPositionTolerance()) { driveVelocityScalar = 0.0; }

        lastSetpoint = new Pose2d(
            goalPose.getTranslation(),
            currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle()
        )
            .transformBy(
                new Transform2d(
                    new Translation2d(translationController.getSetpoint().position, 0.0),
                    new Rotation2d()
                ))
            .getTranslation();

        double thetaVelocity = rotationController.getSetpoint().velocity * ffScaler
                               + rotationController.calculate(
            currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());
        double thetaErrorAbsolute = Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getRadians());
        if (thetaErrorAbsolute < rotationController.getPositionTolerance()) { thetaVelocity = 0.0; }

        Translation2d driveVelocity = new Pose2d(
            new Translation2d(),
            currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle()
        )
            .transformBy(
                new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
            .getTranslation();

        chassis.drive(new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity), true);
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds roboSpeed, Pose2d pose) {
        return new ChassisSpeeds(
            roboSpeed.vxMetersPerSecond * pose.getRotation().getCos()
            - roboSpeed.vyMetersPerSecond * pose.getRotation().getSin(),
            roboSpeed.vyMetersPerSecond * pose.getRotation().getCos()
            + roboSpeed.vxMetersPerSecond * pose.getRotation().getSin(),
            roboSpeed.omegaRadiansPerSecond
        );
    }

    @Override
    public boolean isFinished() {
        return translationController.atGoal() && rotationController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }
}