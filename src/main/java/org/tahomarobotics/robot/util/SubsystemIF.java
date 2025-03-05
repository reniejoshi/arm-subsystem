package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tahomarobotics.robot.util.sysid.SysIdTests;

import java.util.List;

public abstract class SubsystemIF extends SubsystemBase {
    // Initialization

    public SubsystemIF initialize() { return this; }

    public void onDisabledInit() {}

    public void onAutonomousInit() {}

    public void onTeleopInit() {}

    public void onSimulationInit() {}

    // SysId

    public List<SysIdTests.Test> getSysIdTests() {
        return List.of();
    }
}