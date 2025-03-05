package org.tahomarobotics.robot.collector;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.indexer.Indexer;

public class CollectorCommands {
    public static Command createZeroCommand(Collector collector) {
        return collector.runOnce(collector::setZeroingVoltage)
                        .andThen(Commands.waitSeconds(0.1))
                        .andThen(Commands.waitUntil(collector::isDeploymentStopped))
                        .withTimeout(CollectorConstants.DEPLOYMENT_ZEROING_TIMEOUT)
                        .andThen(collector::zero)
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                        .onlyIf(() -> collector.getTargetDeploymentState() == CollectorConstants.TargetDeploymentState.ZEROED);
    }

    public static Command createDeploymentControlCommand(Collector collector) {
        return collector.runOnce(() -> {
            if (collector.isDeploymentStowed()) {
                collector.deploymentTransitionToCollect();
            } else if (collector.isDeploymentCollecting()) {
                collector.deploymentTransitionToStow();
            }
        });
    }

    public static Command createAutoCollectCommand(Collector collector) {
        return collector.runOnce(() -> {
            collector.deploymentTransitionToCollect();
            collector.collectorTransitionToCollecting();
        });
    }

    /** @return On true and on false commands. */
    public static Pair<Command, Command> createCollectorControlCommands(Collector collector) {
        Command onTrue = collector.runOnce(() -> {
            if (collector.isDeploymentCollecting() && collector.isNotHoldingAlgae() && !Indexer.getInstance().isBeanBakeTripped()) {
                collector.collectorTransitionToCollecting();
            }
        });
        Command onFalse = collector.runOnce(() -> {
            if (collector.getTargetCollectorState() != CollectorConstants.TargetCollectorState.HOLDING_ALGAE) {
                collector.collectorTransitionToDisabled();
            }
        });

        return Pair.of(onTrue, onFalse);
    }

    /** @return On true and on false commands. */
    public static Pair<Command, Command> createEjectCommands(Collector collector) {
        Command onTrue = collector.runOnce(collector::transitionToEjecting);
        Command onFalse = collector.runOnce(collector::cancelEjecting);

        return Pair.of(onTrue, onFalse);
    }
}
