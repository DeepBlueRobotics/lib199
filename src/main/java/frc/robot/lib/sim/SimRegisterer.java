package frc.robot.lib.sim;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.IntFunction;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.sim.CallbackStore;
import edu.wpi.first.hal.sim.NotifyCallback;
import edu.wpi.first.hal.sim.PWMSim;
import edu.wpi.first.hal.sim.SimDeviceCallback;
import edu.wpi.first.hal.sim.SimDeviceSim;
import edu.wpi.first.wpilibj.SensorUtil;

public class SimRegisterer {
    
    private static final SimDeviceCallback MISC_DEVICE_CALLBACK = SimRegisterer::callback;
    private static final ArrayDeque<PortDevice> devices = new ArrayDeque<>();
    private static final ArrayDeque<String> miscDevices = new ArrayDeque<>();
    private static final ArrayList<CallbackStore> callbacks = new ArrayList<>();

    static {
        SimUtils.registerSimDeviceCreatedCallback("", MISC_DEVICE_CALLBACK, true);
        registerDeviceType("PWM", SensorUtil.kPwmChannels, PWMSim::new, PWMSim::getInitialized, PWMSim::registerInitializedCallback);
        Simulation.registerPeriodicMethod(SimRegisterer::periodic);
    }

    public static void init() {}

    private static <T> void registerDeviceType(String type, int max, IntFunction<T> newFunc, Function<T, Boolean> isInitalized, CallbackRegisterFunc<T> registerFunc) {
        for(int i = 0; i < max; i++) {
            T obj = newFunc.apply(i);
            if(isInitalized.apply(obj)) {
                callback(type, i, null, null, -1);
                continue;
            }
            final int port = i;
            final int storePos = callbacks.size();
            callbacks.add(registerFunc.registerInitializedCallback(obj, (name, value) -> callback(type, port, name, value, storePos), false));
        }
    }

    public static void periodic() {
        while(!devices.isEmpty()) {
            PortDevice device = devices.poll();
            if(device.type.equals("PWM")) {
                callbacks.add(new PWMSim(device.port).registerSpeedCallback(new WebotsMotorForwarder(Simulation.robot, "PWM_" + device.port), true));
            }
        }
        while(!miscDevices.isEmpty()) {
            String deviceName = miscDevices.poll();
            if(deviceName.startsWith("Talon") || deviceName.startsWith("Victor")) {
                final WebotsMotorForwarder fwdr = new WebotsMotorForwarder(Simulation.robot, deviceName);
            SimUtils.registerValueChangedCallback(new SimDeviceSim(deviceName), "Motor Output",
                (name, handle, readonly, value) -> fwdr.callback(name, value), true);
            }
        }
    }

    private static void callback(String name, int handle) {
        miscDevices.add(name);
    }

    private static void callback(String type, int port, String name, HALValue value, int storePos) {
        devices.add(new PortDevice(type, port));
        if(storePos > -1) {
            callbacks.get(storePos).close();
            callbacks.set(storePos, null);
        }
    }

    private static interface CallbackRegisterFunc<T> {
        CallbackStore registerInitializedCallback(T obj, NotifyCallback callback, boolean initialNotify);
    }

    private static class PortDevice {

        final String type;
        final int port;

        PortDevice(String type, int port) {
            this.type = type;
            this.port = port;
        }

    }

}