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
package org.tahomarobotics.robot.util.motion;


public class SCurveMotionProfile extends MotionProfile {

    public SCurveMotionProfile(double startTime, double startPosition, double endPosition, double startVelocity, double endVelocity, double maxVelocity, double maxAcceleration, double maxJerk) throws MotionProfileException {
        this(startTime, startPosition, endPosition, startVelocity, endVelocity, maxVelocity, maxAcceleration, maxJerk, false, false);
    }

    public SCurveMotionProfile(double startTime, double startPosition, double endPosition, double startVelocity, double endVelocity, double maxVelocity, double maxAcceleration, double maxJerk, boolean rotational, boolean reverse) throws MotionProfileException {
        super(startTime, startPosition, endPosition, startVelocity, endVelocity, maxVelocity, maxAcceleration, maxJerk, rotational, reverse);
    }
    @Override
    protected MotionState[] generatePhases() throws MotionProfileException {

        // TODO: what about start and end velocities?
        if (startVelocity != 0d || endVelocity != 0d) {
            throw new MotionProfileException("S-Curve profiles not defined for with non-zero velocity end points");
        }

        // all velocity and acceleration will be corrected for sign
        double dist = endPosition - startPosition;

        double opp_dist = (Math.abs(dist) - Math.PI*2d) * Math.signum(dist);
        double rotationalDistance = (Math.abs(dist) < Math.abs(opp_dist)) ^ reverse ? dist : opp_dist;

        final double distance = rotational ? rotationalDistance : dist;
        final double abs_distance = Math.abs(distance);
        final double direction = Math.signum(distance);

        // shortest run where max acceleration is not reached (no constant acceleration)
        final double adjustedMaxAcceleration = Math.min(
                maxAcceleration,
                Math.pow(0.5 * abs_distance * maxJerk * maxJerk, 1d/3 ));

        // acceleration ramp time or jerk time
        final double tj = adjustedMaxAcceleration/maxJerk;

        // short run where max velocity cannot be reached (no constant velocity)
        double maxVelocityShort = adjustedMaxAcceleration * ( Math.sqrt(tj * tj / 4 + abs_distance / adjustedMaxAcceleration ) - tj/2);

        final double adjustedMaxVelocity = Math.min(maxVelocity, maxVelocityShort);

        // delta times
        // tj -> ta -> tj -> tv -> tj -> ta -> tj
        // total = 4 tj + 2 ta + tv

        // velocity ramp time or acceleration time
        final double ta = adjustedMaxVelocity / adjustedMaxAcceleration - tj;

        // position ramp time or velocity time or total time
        final double tv = abs_distance / adjustedMaxVelocity - ta - tj - tj;

        MotionState[] phases = new MotionState[8];

        // ramping to max acceleration
        phases[0] = new MotionState()
                .setTime(startTime)
                .setPosition(startPosition)
                .setJerk(maxJerk * direction);

        // start of constant max acceleration
        phases[1] = getPhaseSetpoint(tj, phases[0], new MotionState())
                .setAcceleration(adjustedMaxAcceleration * direction)
                .setJerk(0);

        // ramp acceleration to zero
        phases[2] = getPhaseSetpoint(ta, phases[1], new MotionState())
                .setJerk(-maxJerk * direction);

        // constant velocity
        phases[3] = getPhaseSetpoint(tj, phases[2], new MotionState())
                .setVelocity(adjustedMaxVelocity * direction)
                .setAcceleration(0)
                .setJerk(0);

        // ramping to max reverse acceleration
        phases[4] = getPhaseSetpoint(tv, phases[3], new MotionState())
                .setJerk(-maxJerk * direction);

        // start of constant max reverse acceleration
        phases[5] = getPhaseSetpoint(tj, phases[4], new MotionState())
                .setAcceleration(-adjustedMaxAcceleration * direction)
                .setJerk(0);

        // ramp reverse acceleration to zero
        phases[6] = getPhaseSetpoint(ta, phases[5], new MotionState())
                .setJerk(maxJerk * direction);

        // final
        phases[7] = getPhaseSetpoint(tj, phases[6], new MotionState())
                .setPosition(endPosition)
                .setVelocity(0)
                .setAcceleration(0)
                .setJerk(0);

        return phases;
    }
}
