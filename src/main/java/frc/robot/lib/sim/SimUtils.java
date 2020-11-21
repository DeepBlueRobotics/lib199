package frc.robot.lib.sim;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimValue;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/**
 * Provides utility methods to be used in SimRegister and other simulation code.
 * These are mostly (only) to patch flaws in WPILib for which fixes are still being implemented
 */
public final class SimUtils {
    // Keep Track of Values Watched by Each Value Callback
    private static final HashMap<SimValueCallback, SimValue> valueCallbacks = new HashMap<>();
    // Cache the Previous State of SimValues for New Callbacks
    private static final HashMap<SimValue, HALValue> lastData = new HashMap<>();

    static {
        // Update callbacks as part of the periodic loop
        Simulation.registerPeriodicMethod(SimUtils::periodic);
    }

    // Custom implementation of {@link SimDeviceSim#registerValueChangedCallback}
    public static void registerValueChangedCallback(SimDeviceSim device, String valueName, SimValueCallback callback, boolean initialNotify) {
        // Get the Corresponding SimValue
        SimValue value = device.getValue(valueName);
        // Register this callback
        valueCallbacks.put(callback, value);
        // Fetch the SimValue if we have not yet done so
        lastData.put(value, value.getValue());
        // If requested, notify callback of inital value
        if(initialNotify) {
            // Notify the callback using the cached value
            callback.callback("", -1, false, lastData.get(value));
        }
    }
    
    // Notify registered callbacks of new devices
    public static void periodic() {
        ArrayList<Integer> updatedValues = new ArrayList<>();
        // For all cached data
        lastData.forEach((simValue, value) -> {
            HALValue newValue = simValue.getValue();
            // If the value has changed
            if(newValue.getNativeDouble() != value.getNativeDouble() || newValue.getNativeLong() != value.getNativeLong()) {
                // Update the value and mark it as updated
                lastData.put(simValue, newValue);
                updatedValues.add(simValue.getNativeHandle());
            }
        });
        // For each value callback
        for(SimValueCallback callback : valueCallbacks.keySet()) {
            SimValue value = valueCallbacks.get(callback);
            // If the value has changed
            if(updatedValues.contains(value.getNativeHandle())) {
                // Call the callback
                callback.callback("", -1, false, lastData.get(value));
            }
        }
    }

}