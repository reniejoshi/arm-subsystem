package org.tahomarobotics.robot.util.signals;

import com.ctre.phoenix6.BaseStatusSignal;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public record LoggedStatusSignal(String name, BaseStatusSignal signal) {
    private static BaseStatusSignal[] getBaseSignals(LoggedStatusSignal[] signals) {
        return Arrays.stream(signals).map(LoggedStatusSignal::signal).toArray(BaseStatusSignal[]::new);
    }

    public static void setUpdateFrequencyForAll(LoggedStatusSignal[] signals, double updateFrequency) {
        BaseStatusSignal.setUpdateFrequencyForAll(updateFrequency, getBaseSignals(signals));
    }

    public static void refreshAll(LoggedStatusSignal[] signals) {
        BaseStatusSignal.refreshAll(getBaseSignals(signals));
    }

    public static void waitForAll(double timeout, LoggedStatusSignal[] signals) {
        BaseStatusSignal.waitForAll(timeout, getBaseSignals(signals));
    }

    public static void log(String prefix, LoggedStatusSignal[] signals) {
        prefix += "Status Signals/";

        for (var signal : signals) {
            String prefix_ = prefix + "/" + signal.name + "/";

            Logger.recordOutput(prefix_ + "value", signal.signal().getValueAsDouble());
            Logger.recordOutput(prefix_ + "status", signal.signal().getStatus());
            Logger.recordOutput(prefix_ + "units", signal.signal().getUnits());
        }
    }
}
