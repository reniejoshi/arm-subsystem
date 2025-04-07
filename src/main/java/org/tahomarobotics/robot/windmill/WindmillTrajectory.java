/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.util.motion.MotionProfile;
import org.tahomarobotics.robot.util.motion.MotionState;
import org.tahomarobotics.robot.util.motion.SCurveMotionProfile;
import org.tahomarobotics.robot.util.motion.TrapezoidalMotionProfile;
import org.tinylog.Logger;

import java.util.Optional;

import static org.tahomarobotics.robot.windmill.WindmillConstants.*;

public class WindmillTrajectory {

    protected record WindmillProfile(MotionProfile elev, MotionProfile arm) {
        public double getEndTime() {
            return Math.max(elev.getEndTime(), arm.getEndTime());
        }
    }

    public final String name;
    private final WindmillProfile[] profiles;
    public final WindmillState[] states;

    MotionState elevState = new MotionState();
    MotionState armState = new MotionState();

    private double elevTimeDelay = 0d;
    private double armTimeDelay = 0d;

    public WindmillTrajectory(String name, WindmillState[] states) throws MotionProfile.MotionProfileException {
        this(name, states, false);
    }

    public WindmillTrajectory(String name, WindmillState[] states, WindmillConstraints constraints) throws MotionProfile.MotionProfileException {
        this(name, states, constraints, false);
    }
    public WindmillTrajectory(String name, WindmillState[] states, boolean reverseArm) throws MotionProfile.MotionProfileException {
       this(name, states, WindmillConstraints.CORAL_CONSTRAINTS, reverseArm);
    }

    public WindmillTrajectory(String name, WindmillState[] states, WindmillConstraints constraints, boolean reverseArm) throws MotionProfile.MotionProfileException {
        this.name = name;
        this.states = states;
        if (states == null || states.length < 2) {
            throw new IllegalArgumentException("states either null or less than two");
        }

        profiles = new WindmillProfile[states.length - 1];
        WindmillProfile prior = null;
        for (int i = 0; i < profiles.length; i++) {
            prior = profiles[i] = createProfile(prior, i, states, constraints, reverseArm);
            if (Double.isNaN(prior.getEndTime())) {
                throw new MotionProfile.MotionProfileException("end-time is NaN");
            }
        }


        //printTrajectory();
        adjustDelayForObstructions();
        adjustDelayForMaxDistanceToObstruction();
        logTiming();

    }

    private void logTiming() {
        String delayComp = elevTimeDelay > 0d ? "elev" : "arm";
        double delay = Math.max(elevTimeDelay, armTimeDelay);
        String delayStr = delay > 0d ? String.format(" with %s delay of %5.3f seconds", delayComp, delay) : "";
        Logger.info(String.format("%-30s Duration: %5.3f elev %5.3f arm %5.3f %s.",
                                  name,
                                  getTotalTimeSeconds(),
                                  profiles[profiles.length - 1].elev.getEndTime(),
                                  profiles[profiles.length - 1].arm.getEndTime(),
                                  delayStr));
    }

    private boolean checkProfileWithDelay(double elevDelay, double armDelay) {
        elevTimeDelay = elevDelay;
        armTimeDelay = armDelay;

        double time = 0d;
        while(!sample(time, elevState, armState)) {
            if (isObstructed(elevState.position, armState.position)) {
                return false;
            }
            time += Robot.defaultPeriodSecs;
        }

        return true;
    }

    private void adjustDelayForMaxDistanceToObstruction() {
        double wiggleRoom = profiles[profiles.length - 1].arm.getEndTime() - profiles[profiles.length - 1].elev.getEndTime();
        double elevWiggleRoom = Math.max(elevTimeDelay, wiggleRoom);
        double armWiggleRoom = Math.max(armTimeDelay, -wiggleRoom);

        if (elevWiggleRoom > 0d && armTimeDelay == 0d) {
            // shift or delay elevator for maximum distance to obstructions
            double minCost = 1e9;
            double atDelay = 0d;
            for(double delay = 0d; delay <= elevWiggleRoom; delay += Robot.defaultPeriodSecs) {
                double cost = costFunctionForObstruction(delay, 0d);
                if (cost < minCost) {
                    minCost = cost;
                    atDelay = delay;
                }
            }
            elevTimeDelay = atDelay;
            //Logger.info(String.format("%s: added elevator delay of %6.3f seconds @ cost of %6.3f", name, elevTimeDelay, minCost));

        } else if (armWiggleRoom > 0d && elevTimeDelay == 0d) {
            // shift or delay arm for maximum distance to obstructions
            double minCost = 1e9;
            double atDelay = 0d;
            for(double delay = 0d; delay <= armWiggleRoom; delay += Robot.defaultPeriodSecs) {
                double cost = costFunctionForObstruction(0d, delay);
                if (cost < minCost) {
                    minCost = cost;
                    atDelay = delay;
                }
            }
            armTimeDelay = atDelay;
            //Logger.info(String.format("%s: added arm delay of %6.3f seconds @ cost of %6.3f", name, armTimeDelay, minCost));
        }

    }

    private double distanceFromObstruction(double elevDelay, double armDelay) {
        double distance = 10d;
        double time = 0d;
        while(!sample(time, elevState, armState, elevDelay, armDelay)) {
            double d = yDistanceFromObstruction(elevState.position, armState.position);
            distance = Math.min(distance, d);
            time += Robot.defaultPeriodSecs;
        }
        return distance;
    }

    private double costFunctionForObstruction(double elevDelay, double armDelay) {
        double cost = 0d;
        double time = 0d;
        while(!sample(time, elevState, armState, elevDelay, armDelay)) {
            double d = yDistanceFromObstruction(elevState.position, armState.position);
            cost += d <= 0d ? 1e9 : 1d/d;
            time += Robot.defaultPeriodSecs;
        }
        return cost;
    }

    private void adjustDelayForObstructions() throws MotionProfile.MotionProfileException {
        double maxDelay = 3d;
        for (double delay = 0; delay < maxDelay; delay += Robot.defaultPeriodSecs) {

            // check elevator delay
            if (checkProfileWithDelay(delay, 0d)) {
                elevTimeDelay = delay;
                if (delay > 0d) {
                    //Logger.info(String.format("%s: added elevator delay of %6.3f seconds", name, elevTimeDelay));
                }
                return;
            }
            // check arm delay
            else if (checkProfileWithDelay(0d, delay)) {
                armTimeDelay = delay;
                if (delay > 0d) {
                    //Logger.info(String.format("%s: added arm delay of %6.3f seconds", name, armTimeDelay));
                }
                return;
            }
        }
        throw new MotionProfile.MotionProfileException(name + " Failed to find delay to avoid obstructions");
    }

    private boolean isObstructed(double elev, double arm) {
        double y = elev + WindmillConstants.ARM_LENGTH * Math.sin(arm);
        double x = WindmillConstants.ARM_LENGTH * Math.cos(arm);

        return (y < ROBOT_DECK || (x > ROBOT_COLLECTOR_X && y < ROBOT_COLLECTOR_Y));
    }

    private double yDistanceFromObstruction(double elev, double arm) {
        double y = elev + WindmillConstants.ARM_LENGTH * Math.sin(arm);
        double x = WindmillConstants.ARM_LENGTH * Math.cos(arm);

        return y - (x > ROBOT_COLLECTOR_X ? ROBOT_COLLECTOR_Y : ROBOT_DECK);
    }

    private WindmillProfile createProfile(WindmillProfile prior, int index, WindmillState[] states, WindmillConstraints constraints, boolean reverseArm) throws MotionProfile.MotionProfileException {
        double elevStartTime = 0;
        double armStartTime = 0;
        double elevStartVelocity = 0;
        double armStartVelocity = 0;
        if (prior != null) {
            elevStartTime = prior.elev.getEndTime();
            armStartTime = prior.arm.getEndTime();
            elevStartVelocity = prior.elev.getLastMotionState().velocity;
            armStartVelocity = prior.arm.getLastMotionState().velocity;
        }


        // determine ending velocities
        double elevEndVelocity = 0.0;
        double armEndVelocity = 0.0;

        if (states.length > index + 2) {

            double currentDirection = Math.signum(states[index + 1].elevatorState().heightMeters() - states[index].elevatorState().heightMeters());
            double nextDirection = Math.signum(states[index + 2].elevatorState().heightMeters() - states[index + 1].elevatorState().heightMeters());
            if (currentDirection == nextDirection) {
                elevEndVelocity = constraints.elevMaxVel;
            }

            currentDirection = Math.signum(MathUtil.angleModulus( states[index + 1].armState().angleRadians() - states[index].armState().angleRadians()));
            nextDirection = Math.signum(MathUtil.angleModulus(states[index + 2].armState().angleRadians() - states[index + 1].armState().angleRadians()));
            if (currentDirection == nextDirection) {
                armEndVelocity = constraints.armMaxVel;
            }
        }

        MotionProfile elev = createProfile(
            elevStartTime,
            states[index].elevatorState().heightMeters(),
            states[index + 1].elevatorState().heightMeters(),
            elevStartVelocity, elevEndVelocity,
            constraints.elevMaxVel, constraints.elevMaxAccel, constraints.elevMaxJerk, false, false
        );

        MotionProfile arm = createProfile(
            armStartTime,
            states[index].armState().angleRadians(),
            states[index + 1].armState().angleRadians(),
            armStartVelocity, armEndVelocity,
            constraints.armMaxVel, constraints.armMaxAccel, constraints.armMaxJerk, true, reverseArm
        );

        return new WindmillProfile(elev, arm);
    }

    private MotionProfile createProfile(double startTime, double startPosition, double endPosition, double startVelocity, double endVelocity, double maxVelocity, double maxAcceleration, double maxJerk, boolean rotational, boolean reverse) throws MotionProfile.MotionProfileException {
        return startVelocity == 0d && endVelocity == 0d ?
            new SCurveMotionProfile(startTime, startPosition, endPosition, 0d, 0d, maxVelocity, maxAcceleration, maxJerk, rotational, reverse) :
            new TrapezoidalMotionProfile(startTime, startPosition, endPosition, startVelocity, endVelocity, maxVelocity, maxAcceleration, rotational, reverse);
    }

    private void printTrajectory() {

        boolean done = false;
        System.out.println(name);
        System.out.println("time, elev, arm, obs");
        for (double t = 0d; !done; t += 0.02) {
            done = sample(t, elevState, armState);
            System.out.format("%f, %f, %f, %s%n", t, elevState.position, armState.position, isObstructed(elevState.position, armState.position) ? "true": "false");
        }
    }

    public boolean sample(double time, MotionState elev, MotionState arm) {
        return sample(time, elev, arm, elevTimeDelay, armTimeDelay);
    }

    private boolean sample(double time, MotionState elev, MotionState arm, double elevDelay, double armDelay) {

        double t = Math.max(0d, time - elevDelay);
        boolean elevProfile = true;
        for (WindmillProfile profile : profiles) {
            if (profile.elev.getSetpoint(t, elev)) {
                // got sample (i.e. not at end)
                elevProfile = false;
                break;
            }
            // end of elev profile, check next profile if available
        }

        t = Math.max(0d, time - armDelay);
        boolean armProfile = true;
        for (WindmillProfile profile : profiles) {
            if (profile.arm.getSetpoint(t, arm)) {
                // got sample (i.e. not at end)
                armProfile = false;
                break;
            }
            // end of arm profile, check next profile if available
        }

        return elevProfile && armProfile;
    }



    public WindmillState getInitialState() {

        sample(0d, elevState, armState);

        return new WindmillState(0d,
                                                new WindmillState.ElevatorState(elevState.position, elevState.velocity, elevState.acceleration),
                                                new WindmillState.ArmState(armState.position, armState.velocity, armState.acceleration));
    }

    public double getStartTime() {
        return Math.min(profiles[0].elev.startTime, profiles[0].arm.startTime);
    }

    public double getTotalTimeSeconds() {
        return Math.max(profiles[profiles.length - 1].elev.getEndTime()+elevTimeDelay, profiles[profiles.length - 1].arm.getEndTime()+armTimeDelay);
    }

    public static Optional<WindmillTrajectory> loadTrajectories(Pair<WindmillConstants.TrajectoryState, WindmillConstants.TrajectoryState> fromTo) {
        Logger.info("Loading trajectory {}", fromTo);


        return Optional.empty();
    }

    public record WindmillConstraints(double armMaxVel, double elevMaxVel, double armMaxAccel, double elevMaxAccel, double armMaxJerk, double elevMaxJerk) {
        public static final WindmillConstraints CORAL_CONSTRAINTS = new WindmillConstraints(
            WindmillConstants.ARM_MAX_VELOCITY,
            WindmillConstants.ELEVATOR_MAX_VELOCITY,
            WindmillConstants.ARM_MAX_ACCELERATION,
            WindmillConstants.ELEVATOR_MAX_ACCELERATION,
            WindmillConstants.ARM_MAX_JERK,
            WindmillConstants.ELEVATOR_MAX_JERK);

        public static final WindmillConstraints ALGAE_CONSTRAINTS = new WindmillConstraints(
            WindmillConstants.ARM_ALGAE_MAX_VELOCITY,
            WindmillConstants.ELEVATOR_MAX_VELOCITY,
            WindmillConstants.ARM_ALGAE_MAX_ACCELERATION,
            WindmillConstants.ELEVATOR_MAX_ACCELERATION,
            WindmillConstants.ARM_MAX_JERK,
            WindmillConstants.ELEVATOR_MAX_JERK);

        public static final WindmillConstraints ALGAE_THROW_CONSTRAINTS = new WindmillConstraints(
            WindmillConstants.ARM_ALGAE_THROW_VELOCITY,
            WindmillConstants.ELEVATOR_MAX_VELOCITY,
            WindmillConstants.ARM_MAX_ACCELERATION,
            WindmillConstants.ELEVATOR_MAX_ACCELERATION,
            WindmillConstants.ARM_MAX_JERK,
            WindmillConstants.ELEVATOR_MAX_JERK);

    }
}
