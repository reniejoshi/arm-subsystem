package org.tahomarobotics.robot.windmill.commands;

import edu.wpi.first.math.Pair;
import org.tahomarobotics.robot.util.ImmutableLazyOptionalMap;
import org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState;
import org.tahomarobotics.robot.windmill.WindmillTrajectory;
import org.tinylog.Logger;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import static org.tahomarobotics.robot.windmill.WindmillTrajectory.BEEF_SAVE_DIR;

public class WindmillTrajectories {
    public final static ImmutableLazyOptionalMap<Pair<TrajectoryState, TrajectoryState>, WindmillTrajectory> trajectories =
        new ImmutableLazyOptionalMap<>(
            indexTrajectoryFiles(),
            WindmillTrajectory::loadFromFromTo
        );

    public static Optional<WindmillTrajectory> getTrajectory(TrajectoryState from, TrajectoryState to) {
        return trajectories.get(Pair.of(from, to));
    }

    static {
        trajectories.keys().forEach(trajectories::get);
    }

    public static void initialize() {}

    /**
     * Indexes `deploy/beef` into pairs of from-to trajectories corresponding to their file names.
     *
     * @return A list of from-to state pairs
     */
    public static List<Pair<TrajectoryState, TrajectoryState>> indexTrajectoryFiles() {
        String[] files = BEEF_SAVE_DIR.list();

        if (files == null) {
            Logger.error("Error while indexing trajectory files!");
            return List.of();
        }

        return (Arrays.stream(files))
            .flatMap(fileName -> {
                Optional<Pair<TrajectoryState, TrajectoryState>> transition = Optional.empty();
                block:
                {
                    if (!fileName.endsWith(".traj")) { break block; }

                    var parts = fileName.substring(0, fileName.length() - ".traj".length()).split("_TO_");

                    if (parts.length != 2) { break block; }

                    List<TrajectoryState> states;
                    try {
                        states = Arrays.stream(parts).map(TrajectoryState::valueOf).toList();
                    } catch (IllegalArgumentException e) {
                        break block;
                    }

                    transition = Optional.of(Pair.of(states.get(0), states.get(1)));
                }


                if (transition.isEmpty()) {
                    Logger.warn("File '{}' has a malformed name, ignoring...", fileName);
                }

                return transition.stream();
            }).toList();
    }
}
