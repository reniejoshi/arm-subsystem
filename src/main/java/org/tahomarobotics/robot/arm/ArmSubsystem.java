package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;

public class ArmSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX armMotor = new TalonFX(RobotMap.ARM_MOTOR);

    // Control requests
    PositionVoltage posControl = new PositionVoltage(0);
    VoltageOut voltControl = new VoltageOut(0);

    // Status signals
    private final StatusSignal<Angle> armMotorPosition = armMotor.getPosition();

    // Target position
    private double targetPosition = 0;

    public void setArmPosition(DoubleSupplier rightYSupplier) {
        double y = rightYSupplier.getAsDouble();
        Logger.recordOutput("Arm/Right Y Axis", y);

        double increase = y * ArmConstants.INCREMENT;
        targetPosition = MathUtil.clamp(
            targetPosition + increase,
            ArmConstants.MIN_POSITION,
            ArmConstants.MAX_POSITION);
        armMotor.setControl(posControl.withPosition(Degrees.of(targetPosition)));
    }

    @Override
    public void subsystemPeriodic() {
        armMotorPosition.refresh();

        Logger.recordOutput("Arm/Arm Motor Position", armMotorPosition.getValue());
        Logger.recordOutput("Arm/Target Arm Position", targetPosition);
    }
}
