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
import static org.tahomarobotics.robot.arm.ArmConstants.ZEROING_VOLTAGE;
import static org.tahomarobotics.robot.arm.ArmConstants.ZERO_POSITION;

public class ArmSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX armMotor = new TalonFX(RobotMap.ARM_MOTOR);

    // Control requests
    PositionVoltage posControl = new PositionVoltage(0);
    VoltageOut voltControl = new VoltageOut(0);

    // Status signals
    private final StatusSignal<Angle> armMotorPosition = armMotor.getPosition();

    public void setArmPosition(DoubleSupplier rightYSupplier) {
        double y = rightYSupplier.getAsDouble();
        Logger.recordOutput("Arm/Right Y Axis", y);

        double targetPositionDouble = armMotorPosition.getValueAsDouble();
        if (y > 0) {
            targetPositionDouble += ArmConstants.INCREMENT;
        } else if (y < 0) {
            targetPositionDouble -= ArmConstants.INCREMENT;
        }

        Angle targetPosition = Degrees.of(MathUtil.clamp(targetPositionDouble, ArmConstants.MIN_POSITION, ArmConstants.MAX_POSITION));
        armMotor.setControl(posControl.withPosition(targetPosition));
        Logger.recordOutput("Arm/Target Arm Position", targetPosition);
    }
    public void applyZeroVoltage(double volts){
        armMotor.setControl(voltControl.withOutput(ZEROING_VOLTAGE));
    }
    public void setZeroPosition(){
        armMotor.setControl(voltControl.withOutput(ZERO_POSITION));
        armMotor.setPosition(ZERO_POSITION);
    }

    @Override
    public void subsystemPeriodic() {
        armMotorPosition.refresh();

        Logger.recordOutput("Arm/Arm Motor Position", armMotorPosition.getValue());
    }
}
