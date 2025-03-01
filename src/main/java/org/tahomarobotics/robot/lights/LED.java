package org.tahomarobotics.robot.lights;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.util.SubsystemIF;

/*
LED Channels
 - A = green
 - B = red
 - C = blue
 */
public class LED extends SubsystemIF {
    private static final LED INSTANCE = new LED();

    private final CANifier canifier = new CANifier(RobotMap.LED);

    private LED() {
    }

    @Override
    public SubsystemIF initialize() {
        setColor(Color.kGold);
        return this;
    }

    public static LED getInstance() {
        return INSTANCE;
    }

    public void updateLEDColor() {
        if (RobotState.isEnabled()) {
            switch (Collector.getInstance().getCollectionMode()) {
                case ALGAE -> setColor(new Color(0, 1, 0.25));
                case CORAL -> setColor(Color.kWhite);
                default -> setColor(Color.kGold);
            }
        } else {
            setColor(Color.kGold);
        }
    }

    public void setColor(Color color) {
        canifier.setLEDOutput(color.red, CANifier.LEDChannel.LEDChannelB);
        canifier.setLEDOutput(color.green, CANifier.LEDChannel.LEDChannelA);
        canifier.setLEDOutput(color.blue, CANifier.LEDChannel.LEDChannelC);
    }

    @Override
    public void periodic() {
        updateLEDColor();
    }
}