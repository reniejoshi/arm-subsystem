package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class ArmSubsystem extends AbstractSubsystem {
    TalonFX armMotor = new TalonFX(RobotMap.ARM_MOTOR);
    PositionVoltage posControl = new PositionVoltage(0);
    VoltageOut voltControl = new VoltageOut(0);

    @Override
    public void subsystemPeriodic(){
    }

}
