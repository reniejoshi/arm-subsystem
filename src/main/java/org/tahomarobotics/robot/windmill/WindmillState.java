package org.tahomarobotics.robot.windmill;

import edu.wpi.first.math.util.Units;

public record WindmillState(
    double timeSeconds,
    ElevatorState elevatorState,
    ArmState armState
) {
    public record ElevatorState(
        double heightMeters,
        double velocityMetersPerSecond,
        double accelerationMetersPerSecondSquared
    ) {
        public static ElevatorState fromPrevious(
            double deltaTimeSeconds, double heightMeters, ElevatorState previousState
        ) {
            double velocityMetersPerSecond = (heightMeters - previousState.heightMeters) / deltaTimeSeconds;
            double accelerationMetersPerSecondSquared = (velocityMetersPerSecond - previousState.velocityMetersPerSecond) / deltaTimeSeconds;

            return new ElevatorState(heightMeters, velocityMetersPerSecond, accelerationMetersPerSecondSquared);
        }
    }

    public record ArmState(
        double angleRadians,
        double velocityRadiansPerSecond,
        double accelerationRadiansPerSecondSquared
    ) {
        public static ArmState fromPrevious(double deltaTimeSeconds, double angleRadians, ArmState previousState) {
            double velocityRadiansPerSecond = (angleRadians - previousState.angleRadians) / deltaTimeSeconds;
            double accelerationRadiansPerSecondSquared = (velocityRadiansPerSecond - previousState.velocityRadiansPerSecond) / deltaTimeSeconds;

            return new ArmState(angleRadians, velocityRadiansPerSecond, accelerationRadiansPerSecondSquared);
        }
    }

    public static WindmillState fromPrevious(
        double timeSeconds, double elevatorHeightMeters, double armAngleRadians, WindmillState previousState
    ) {
        ElevatorState elevatorState;
        ArmState armState;
        //@formatter:off
        if (previousState != null) blk: {
            double deltaTimeSeconds = timeSeconds - previousState.timeSeconds;
            if (deltaTimeSeconds <= 0) break blk;

            elevatorState = ElevatorState.fromPrevious(deltaTimeSeconds, elevatorHeightMeters, previousState.elevatorState);
            armState = ArmState.fromPrevious(deltaTimeSeconds, armAngleRadians, previousState.armState);

            return new WindmillState(timeSeconds, elevatorState, armState);
        } //@formatter:on

        elevatorState = new ElevatorState(elevatorHeightMeters, 0, 0);
        armState = new ArmState(armAngleRadians, 0, 0);

        return new WindmillState(timeSeconds, elevatorState, armState);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof WindmillState other) {
            return Math.abs(elevatorState.heightMeters - other.elevatorState.heightMeters) < 0.01 &&
                   Math.abs(armState.angleRadians - other.armState.angleRadians) < Units.degreesToRadians(2);
        }
        return false;
    }
}
