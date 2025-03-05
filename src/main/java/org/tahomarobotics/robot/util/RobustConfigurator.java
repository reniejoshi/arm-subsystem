package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.tinylog.Logger;

import java.util.function.Consumer;
import java.util.function.Supplier;

@SuppressWarnings("UnusedReturnValue")
public class RobustConfigurator {
    /**
     * Number of configuration attempts.
     */
    private static final int RETRIES = 5;

    // Retrying Configurator

    /**
     * Attempts to run the configuration function until success or RETRIES.
     *
     * @param specifier Specifier for the device(s)
     * @param config    Configuration function
     *
     * @return Resulting status code
     */
    @SuppressWarnings("SameParameterValue")
    private static StatusCode tryConfigure(String specifier, Supplier<StatusCode> config) {
        StatusCode statusCode = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < RETRIES; i++) {
            statusCode = config.get();
            if (statusCode.isOK()) {
                Logger.info("Successfully configured {} in {} attempt{}!", specifier, i + 1, i == 0 ? "" : "s");
                break;
            } else if (statusCode.isWarning()) {
                Logger.warn(
                    "[{}/{}] Configuring {} returned warning status code: {}, retrying...", i + 1, RETRIES, specifier,
                    statusCode
                );
            } else {
                Logger.error(
                    "[{}/{}] Configuring {} returned error status code: {}, retrying...", i + 1, RETRIES, specifier,
                    statusCode
                );
            }
        }
        return statusCode;
    }

    // Device Configurators

    /**
     * Attempts to configure a TalonFX.
     *
     * @param deviceName    Name of the device
     * @param motor         The motor
     * @param configuration Configuration to apply
     *
     * @return The resulting status code
     */
    public static StatusCode tryConfigureTalonFX(String deviceName, TalonFX motor, TalonFXConfiguration configuration) {
        return tryConfigure("TalonFX '" + deviceName + "'", () -> motor.getConfigurator().apply(configuration));
    }

    /**
     * Attempts to modify the configuration of a TalonFX.
     *
     * @param deviceName   Name of the device
     * @param motor        The motor
     * @param modification Modification to apply
     *
     * @return The resulting status code
     */
    public static StatusCode tryModifyTalonFX(
        String deviceName, TalonFX motor, Consumer<TalonFXConfiguration> modification) {
        var config = new TalonFXConfiguration();
        motor.getConfigurator().refresh(config);
        modification.accept(config);

        return tryConfigureTalonFX(deviceName, motor, config);
    }

    /**
     * Attempts to configure a CANcoder.
     *
     * @param deviceName    Name of the device
     * @param encoder       The encoder
     * @param configuration Configuration to apply
     *
     * @return The resulting status code
     */
    public static StatusCode tryConfigureCANcoder(
        String deviceName, CANcoder encoder, CANcoderConfiguration configuration) {
        return tryConfigure("CANcoder '" + deviceName + "'", () -> encoder.getConfigurator().apply(configuration));
    }

    /**
     * Attempts to modify the configuration of a CANcoder.
     *
     * @param deviceName   Name of the device
     * @param encoder      The encoder
     * @param modification Modification to apply
     *
     * @return The resulting status code
     */
    public static StatusCode tryModifyCANcoder(
        String deviceName, CANcoder encoder, Consumer<CANcoderConfiguration> modification) {
        var config = new CANcoderConfiguration();
        encoder.getConfigurator().refresh(config);
        modification.accept(config);

        return tryConfigureCANcoder(deviceName, encoder, config);
    }

    // Helper Methods

    /**
     * Attempts to set the angular offset of a CANcoder.
     *
     * @param deviceName Name of the device
     * @param encoder    The encoder
     * @param offset     The new offset of the CANcoder in <strong>rotations</strong>
     *
     * @return The resulting status code
     */
    public static StatusCode trySetCANcoderAngularOffset(String deviceName, CANcoder encoder, double offset) {
        return tryModifyCANcoder(deviceName, encoder, e -> e.MagnetSensor.MagnetOffset = offset);
    }

    /**
     * Attempts to set the neutral mode of a TalonFX.
     *
     * @param deviceName Name of the device
     * @param motor      The motor
     * @param mode       The new neutral mode
     *
     * @return The resulting status code
     */
    public static StatusCode trySetMotorNeutralMode(String deviceName, TalonFX motor, NeutralModeValue mode) {
        return tryModifyTalonFX(deviceName, motor, m -> m.MotorOutput.NeutralMode = mode);
    }
}