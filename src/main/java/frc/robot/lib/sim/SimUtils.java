package frc.robot.lib.sim;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimValue;
import edu.wpi.first.hal.sim.SimDeviceCallback;
import edu.wpi.first.hal.sim.SimDeviceSim;
import edu.wpi.first.hal.sim.SimValueCallback;
import edu.wpi.first.hal.sim.mockdata.SimDeviceDataJNI;

//Provides utility methods to be used in SimRegister and other simulation code.
//These are mostly to patch flaws in WPILib for which fixes are still being implemented
public final class SimUtils {

    //Map Device Callbacks to their Prefixes
    private static final HashMap<SimDeviceCallback, String> prefixes = new HashMap<>();
    //Keep Track of Devices Known to Each Device Callback
    private static final HashMap<SimDeviceCallback, ArrayList<String>> knownDevices = new HashMap<>();
    //Keep Track of Values Watched by Each Value Callback
    private static final HashMap<SimValueCallback, SimValue> valueCallbacks = new HashMap<>();
    //Cache the Previous State of SimValues for New Callbacks
    private static final HashMap<SimValue, HALValue> lastData = new HashMap<>();

    static {
        //Update callbacks as part of the periodic loop
        Simulation.registerPeriodicMethod(SimUtils::periodic);
    }

    //Custom implementations of WPILib methods. These are required due to a bug in WPILib which hinderes their functionality (https://github.com/wpilibsuite/allwpilib/issues/2832)
    //The origional WPILib methods produce a crash in native code when called
    //To make migration to the WPILib methods easier, these methods have similar type signatures to their corresponding WPILib methods
    //Notable differences:
    //The WPILib methods are called at the time the Device or Value is created. These methods work by contnually polling WPILib for changes, so they notify their callbacks during the robot's periodic loop
    //The value name and readOnly state cannot be obtained from the value object and the handle is ambiguous as to whether its a device or value handle, so value callbacks are called with "null" versions of these arguments

    //Custom implementation of {@link SimDeviceSim#registerSimDeviceCreatedCallback}
    public static void registerSimDeviceCreatedCallback(String prefix, SimDeviceCallback callback, boolean initialNotify) {
        prefixes.put(callback, prefix);
        knownDevices.put(callback, new ArrayList<>());
    }

    //Custom implementation of {@link SimDeviceSim#registerValueChangedCallback}
    public static void registerValueChangedCallback(SimDeviceSim device, String valueName, SimValueCallback callback, boolean initialNotify) {
        //Get the Corresponding SimValue
        SimValue value = device.getValue(valueName);
        //Register this callback
        valueCallbacks.put(callback, value);
        //Fetch the SimValue if we have not yet done so
        lastData.put(value, value.getValue());
        //If requested, notify callback of inital value
        if(initialNotify) {
            //Notify the callback using the cached value
            callback.callback("", -1, false, lastData.get(value));
        }
    }

    //Notify callback and mark the device as known to the callback
    private static final Consumer<SimDeviceInfo> performDeviceCallback(SimDeviceCallback callback) { return (info) -> {
        callback.callback(info.name, info.handle);knownDevices.get(callback).add(info.name);};}
    
    //Notify registered callbacks of new devices
    public static void periodic() {
        //Map existing devices to their accessible info counterparts and store them in a new list
        List<SimDeviceInfo> devices = Arrays.stream(SimDeviceSim.enumerateDevices("")).map(SimDeviceInfo::new).collect(Collectors.toList());
        //For each device callback
        for(SimDeviceCallback callback : prefixes.keySet()) {
            String prefix = prefixes.get(callback);
            ArrayList<String> kDevs = knownDevices.get(callback);
            //For each device
            devices.stream()
            //If the callback has not already been not already been notifed of this device
            .filter((info) -> !kDevs.contains(info.name))
            //And the device name starts with the prefix
            .filter((info) -> info.name.startsWith(prefix))
            //Call the callback
            .forEach(performDeviceCallback(callback));
        }
        ArrayList<Integer> updatedValues = new ArrayList<>();
        //For all cached data
        lastData.forEach((simValue, value) -> {
            HALValue newValue = simValue.getValue();
            //If the value has changed
            if(newValue.getNativeDouble() != value.getNativeDouble() || newValue.getNativeLong() != value.getNativeLong()) {
                //Update the value and mark it as updated
                lastData.put(simValue, newValue);
                updatedValues.add(simValue.getNativeHandle());
            }
        });
        //For each value callback
        for(SimValueCallback callback : valueCallbacks.keySet()) {
            SimValue value = valueCallbacks.get(callback);
            //If the value has changed
            if(updatedValues.contains(value.getNativeHandle())) {
                //Call the callback
                callback.callback("", -1, false, lastData.get(value));
            }
        }
    }

    //All of this code is for accessing protected WPILib fields (See comment below)

    public static <T> T getPrivateField(Object obj, String fieldName, ExceptionBiFunction<Field, Object, T> getFunc) {
        try {
            Field field = obj.getClass().getDeclaredField(fieldName);
            field.setAccessible(true);
            return getFunc.apply(field, obj);
        } catch(Exception e) {
            return null;
        }
    }

    //These classes are used to access package fields of their corresponding WPILib classes until the issue (#2526) is fixed
    public static final class SimDeviceInfo {
        public final String name;
        public final int handle;

        public SimDeviceInfo(SimDeviceDataJNI.SimDeviceInfo info) {
          this.name = (String)getPrivateField(info, "name", Field::get);
          this.handle = getPrivateField(info, "handle", Field::getInt);
        }
    }

    public static class SimValueInfo {
        public final String name;
        public final int handle;
        public final boolean readonly;
        public final HALValue value;
        public SimValueInfo(SimDeviceDataJNI.SimValueInfo info) {
            this.name = (String)getPrivateField(info, "name", Field::get);
            this.handle = getPrivateField(info, "handle", Field::getInt);
            this.readonly = getPrivateField(info, "readonly", Field::getBoolean);
            this.value = (HALValue)getPrivateField(info, "value", Field::get);
        }
    }

    public static interface ExceptionBiFunction<T, U, R> {
        public R apply(T arg1, U arg2) throws Exception;
    }

}