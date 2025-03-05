package org.tahomarobotics.robot.climber.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.climber.Climber;
import org.tahomarobotics.robot.climber.ClimberConstants;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorConstants;
import org.tahomarobotics.robot.windmill.Windmill;

public class ClimberCommands {
    private static final Climber climber = Climber.getInstance();

    public static Command createZeroCommand(Climber climber) {
        return climber.runOnce(climber::zeroPosition)
                      .andThen(Commands.waitUntil(
                          () ->
                              Collector.getInstance().getTargetDeploymentState() == CollectorConstants.TargetDeploymentState.CORAL_COLLECT ||
                              Collector.getInstance().getTargetDeploymentState() == CollectorConstants.TargetDeploymentState.ALGAE_COLLECT))
                      .andThen(climber.runOnce(climber::stow))
                      .onlyIf(() -> climber.getClimbState() == Climber.ClimberState.ZEROED);
    }

    public static Command getClimberCommand() {
        return Commands.deferredProxy(
            () ->
                switch (climber.getClimbState()) {
                    case STOWED -> Commands.runOnce(climber::deploy)
                                           .andThen(Commands.runOnce(() -> Windmill.getInstance().setArmPosition(0.29 + Units.degreesToRotations(15))))
                                           .andThen(Commands.waitUntil(Windmill.getInstance()::isArmAtPosition))
                                           .andThen(Commands.runOnce(() -> Windmill.getInstance().setElevatorHeight(0.005)));
                    case DEPLOYED -> Commands.runOnce(climber::climb)
                                             .andThen(Commands.runOnce(climber::deploySolenoid))
                                             .andThen(Commands.waitUntil(climber::isAtTargetPosition))
                                             .andThen(Commands.runOnce(climber::disableClimberMotors))
                                             .andThen(Commands.waitSeconds(ClimberConstants.RATCHET_SOLENOID_DEPLOY_TIME))
                                             .andThen(Commands.runOnce(climber::disableSolenoid));
                    case CLIMBED -> Commands.runOnce(climber::stow);
                    case ZEROED -> Commands.none();
                }
        );
    }
}
