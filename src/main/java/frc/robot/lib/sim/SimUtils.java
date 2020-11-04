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
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class SimUtils {

    private static final HashMap<SimDeviceCallback, String> prefixes = new HashMap<>();
    private static final HashMap<SimDeviceCallback, ArrayList<String>> knownDevices = new HashMap<>();
    private static final HashMap<SimValueCallback, SimValue> valueCallbacks = new HashMap<>();
    private static final HashMap<SimValue, HALValue> lastData = new HashMap<>();

    static {
        registerPeriodicMethod(SimUtils::periodic);
    }

    public static void registerPeriodicMethod(Runnable method) {
        new Subsystem() {
            @Override
            public void periodic() {
                method.run();
            }
        }.register();
    }

    public static void registerSimDeviceCreatedCallback(String prefix, SimDeviceCallback callback, boolean initialNotify) {
        prefixes.put(callback, prefix);
        knownDevices.put(callback, new ArrayList<>());
    }

    public static void registerValueChangedCallback(SimDeviceSim device, String valueName, SimValueCallback callback, boolean initialNotify) {
        SimValue value = device.getValue(valueName);
        valueCallbacks.put(callback, value);
        if(initialNotify) {
            if(!lastData.containsKey(value)) {
                lastData.put(value, value.getValue());
            }
            callback.callback("", -1, false, lastData.get(value));
        }
    }

    private static final Consumer<SimDeviceInfo> performDeviceCallback(SimDeviceCallback callback) { return (info) -> {
        callback.callback(info.name, info.handle);knownDevices.get(callback).add(info.name);};}
    public static void periodic() {
        List<SimDeviceInfo> devices = Arrays.stream(SimDeviceSim.enumerateDevices("")).map(SimDeviceInfo::new).collect(Collectors.toList());
        for(SimDeviceCallback callback : prefixes.keySet()) {
            String prefix = prefixes.get(callback);
            ArrayList<String> kDevs = knownDevices.get(callback);
            devices.stream().filter((info) -> !kDevs.contains(info.name)).filter((info) -> info.name.startsWith(prefix)).forEach(performDeviceCallback(callback));
        }
        ArrayList<Integer> updatedValues = new ArrayList<>();
        lastData.forEach((simValue, value) -> {
            HALValue newValue = simValue.getValue();
            if(newValue.getNativeDouble() != value.getNativeDouble() || newValue.getNativeLong() != value.getNativeLong()) {
                lastData.put(simValue, newValue);
                updatedValues.add(simValue.getNativeHandle());
            }
        });
        for(SimValueCallback callback : valueCallbacks.keySet()) {
            SimValue value = valueCallbacks.get(callback);
            if(updatedValues.contains(value.getNativeHandle())) {
                callback.callback("", -1, false, lastData.get(value));
            }
        }
    }

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