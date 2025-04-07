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

import edu.wpi.first.math.MathUtil;

public class TrapezoidalMotionProfile extends MotionProfile {

	public TrapezoidalMotionProfile(double startTime, double startPosition, double endPosition, double startVelocity, double endVelocity, double maxVelocity, double maxAcceleration) throws MotionProfileException {
		super(startTime, startPosition, endPosition, startVelocity, endVelocity, maxVelocity, maxAcceleration, 0, false, false);
	}
	public TrapezoidalMotionProfile(double startTime, double startPosition, double endPosition, double startVelocity, double endVelocity, double maxVelocity, double maxAcceleration, boolean rotational, boolean reverse) throws MotionProfileException {
		super(startTime, startPosition, endPosition, startVelocity, endVelocity, maxVelocity, maxAcceleration, 0, rotational, reverse);
	}

	@Override
	protected MotionState[] generatePhases() throws MotionProfileException {

		// all velocity and acceleration will be corrected for sign
		double dist = endPosition - startPosition;

		double opp_dist = (Math.abs(dist) - Math.PI*2d) * Math.signum(dist);
		double rotationalDistance = (Math.abs(dist) < Math.abs(opp_dist)) ^ reverse ? dist : opp_dist;

		final double distance = rotational ? rotationalDistance : dist;
		final double abs_distance = Math.abs(distance);
		final double direction = Math.signum(distance);

		double start_velocity = Math.abs(startVelocity);
		double end_velocity = Math.abs(endVelocity);

		double max_velocity = Math.min(
			maxVelocity,
			Math.sqrt(abs_distance * maxAcceleration + startVelocity * startVelocity / 2 + endVelocity * endVelocity / 2)
		);


		if (end_velocity > max_velocity) {
			max_velocity = Math.sqrt(abs_distance * maxAcceleration * 2 + start_velocity * start_velocity);
			end_velocity = max_velocity;
		}

		final double ta = Math.max(0, (max_velocity - start_velocity) / (maxAcceleration + 1.0e-6));
		final double td = Math.max(0, (max_velocity - end_velocity) / (maxAcceleration + 1.0e-6));
		double distAccel = start_velocity * ta + 0.5 * maxAcceleration * ta * ta;
		double distDecel = max_velocity * td - 0.5 * maxAcceleration * td * td;
		final double tv = (abs_distance - distAccel - distDecel) / (max_velocity + 1e-6);
		if (tv < 0) {
			throw new MotionProfileException("distance is too short for the deceleration");
		}

		double max_acceleration = direction * maxAcceleration;
		max_velocity *= direction;
		start_velocity *= direction;
		end_velocity *= direction;

		MotionState[] phases = new MotionState[4];

		// initial state
		MotionState initial = phases[0] = new MotionState()
				.setTime(startTime)
				.setPosition(startPosition)
				.setVelocity(start_velocity)
				.setAcceleration(max_acceleration);
		
		// end of constant acceleration
		initial = phases[1] = getPhaseSetpoint(ta, initial, new MotionState())
				.setVelocity(max_velocity)
				.setAcceleration(0);
		
		// end of constant velocity
		initial = phases[2] = getPhaseSetpoint(tv, initial, new MotionState())
				.setAcceleration(-max_acceleration);

		// end of constant deceleration
		phases[3] = getPhaseSetpoint(td, initial, new MotionState())
				.setVelocity(end_velocity)
				.setPosition(endPosition)
				.setAcceleration(0);

		return phases;
	}
}
