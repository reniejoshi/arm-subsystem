package org.tahomarobotics.robot.grabber;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;

public class GrabberCommands {
    public static Pair<Command, Command> createGrabberCommands(Grabber grabber) {
        Command onTrue = grabber.runOnce(grabber::transitionToCollecting);
        Command onFalse = grabber.runOnce(grabber::transitionToDisabled);

        return Pair.of(onTrue, onFalse);
    }

    public static Pair<Command, Command> createGrabberScoringCommands(Grabber grabber) {
        Command onTrue = grabber.runOnce(grabber::transitionToScoring);
        Command onFalse = grabber.runOnce(grabber::transitionToDisabled).onlyIf(() -> !grabber.collectionTimer.isRunning());

        return Pair.of(onTrue, onFalse);
    }
}
