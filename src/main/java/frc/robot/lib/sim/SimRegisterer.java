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

//Performs automatic registration of callbacks detecting both the initalization of new devices as well as data callbacks for devices such as Motors, Gyros, etc.
public class SimRegisterer {
    
    private static final SimDeviceCallback MISC_DEVICE_CALLBACK = SimRegisterer::callback;
    private static final ArrayDeque<PortDevice> devices = new ArrayDeque<>();
    private static final ArrayDeque<String> miscDevices = new ArrayDeque<>();
    private static final ArrayList<CallbackStore> callbacks = new ArrayList<>();

    static {
        //Register Initalized Callbacks for Misc Devices
        SimUtils.registerSimDeviceCreatedCallback("", MISC_DEVICE_CALLBACK, true);
        //Register Initalized Callbacks for PWM Devices
        registerDeviceType("PWM", SensorUtil.kPwmChannels, PWMSim::new, PWMSim::getInitialized, PWMSim::registerInitializedCallback);
        //Run our periodic method in the periodic loop
        Simulation.registerPeriodicMethod(SimRegisterer::periodic);
    }

    public static void init() {}

    //Register callbacks for known devices on Non-Can ports
    private static <T> void registerDeviceType(String type, int max, IntFunction<T> newFunc, Function<T, Boolean> isInitalized, CallbackRegisterFunc<T> registerFunc) {
        //For all valid device ports
        for(int i = 0; i < max; i++) {
            //Create a sim for this port
            T obj = newFunc.apply(i);
            //If already initalized call callback now
            if(isInitalized.apply(obj)) {
                callback(type, i, null, null, -1);
                continue;
            }
            //Otherwise register an initialization callback
            final int port = i;
            final int storePos = callbacks.size();
            callbacks.add(registerFunc.registerInitializedCallback(obj, (name, value) -> callback(type, port, name, value, storePos), false));
        }
    }

    //The registered initalization callbacks place new devices into a queue.
    //This method processes that queue and tries to link any known devices to Webots
    public static void periodic() {
        while(!devices.isEmpty()) {
            PortDevice device = devices.poll();
            if(device.type.equals("PWM")) {
                //If a new PWM device has been initalized, attempt to link it to a Webots Motor
                //Register a speed callback on this device
                callbacks.add(new PWMSim(device.port).registerSpeedCallback(
                    //Call a motor forwarder for a callback
                    new WebotsMotorForwarder(Simulation.robot, "PWM_" + device.port),
                    //Initalize with current speed
                    true));
            }
        }
        while(!miscDevices.isEmpty()) {
            String deviceName = miscDevices.poll();
            if(deviceName.startsWith("Talon") || deviceName.startsWith("Victor")) {
                //If a new Talon or Victor has been initalized, attempt to link it to a Webots Motor
                //Create a WebotsMotorForwarder for this motor
                final WebotsMotorForwarder fwdr = new WebotsMotorForwarder(Simulation.robot, deviceName);
                //Register a callback for when the Motor Output changes
                SimUtils.registerValueChangedCallback(new SimDeviceSim(deviceName), "Motor Output",
                    //Call the callback function
                    (name, handle, readonly, value) -> fwdr.callback(name, value),
                    //Initalize with current speed
                    true);
            }
            if(deviceName.startsWith("navX")) {
                //If a navX is registered, try to link its SimDevice to the Webots robot
                MockGyro.linkGyro();
            }
        }
    }

    //Callback for when a Miscellaneous Device is registered
    private static void callback(String name, int handle) {
        miscDevices.add(name);
    }

    //Callback for when a known device type is registered on a Non-Can port
    private static void callback(String type, int port, String name, HALValue value, int storePos) {
        devices.add(new PortDevice(type, port));
        if(storePos > -1) {
            callbacks.get(storePos).close();
            callbacks.set(storePos, null);
        }
    }

    //Func template for callbacks
    private static interface CallbackRegisterFunc<T> {
        CallbackStore registerInitializedCallback(T obj, NotifyCallback callback, boolean initialNotify);
    }

    //Mapping between device types and ports
    private static class PortDevice {

        final String type;
        final int port;

        PortDevice(String type, int port) {
            this.type = type;
            this.port = port;
        }

    }

}