package frc.robot.lib.sim;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.IntFunction;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.simulation.NotifyCallback;
import edu.wpi.first.hal.simulation.SimDeviceCallback;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.simulation.CallbackStore;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

// Performs automatic registration of callbacks detecting both the initalization of new devices as well as data callbacks for devices such as Motors, Gyros, etc.
// This allows us to automatically link these devices to Webots, reducing the amount of code we would have to change from a standard robot project
public class SimRegisterer {
    
    private static final SimDeviceCallback MISC_DEVICE_CALLBACK = SimRegisterer::callback;
    private static final ArrayList<CallbackStore> callbacks = new ArrayList<>();

    static {
        // Register Initalized Callbacks for Misc Devices
        SimUtils.registerSimDeviceCreatedCallback("", MISC_DEVICE_CALLBACK, true);
        // Register Initalized Callbacks for PWM Devices
        registerDeviceType("PWM", SensorUtil.kPwmChannels, PWMSim::new, PWMSim::getInitialized, PWMSim::registerInitializedCallback);
    }

    // Initalize SimRegisterer. This method exists to ensure that the static block is called
    public static void init() {}

    // Register callbacks for known devices on Non-Can ports
    private static <T> void registerDeviceType(String type, int max, IntFunction<T> newFunc, Function<T, Boolean> isInitalized, CallbackRegisterFunc<T> registerFunc) {
        // For all valid device ports
        for(int i = 0; i < max; i++) {
            // Create a sim for this port
            T obj = newFunc.apply(i);
            // If already initalized call callback now
            if(isInitalized.apply(obj)) {
                callback(type, i, null, null, -1);
                continue;
            }
            // Otherwise register an initialization callback
            final int port = i;
            final int storePos = callbacks.size();
            callbacks.add(registerFunc.registerInitializedCallback(obj, (name, value) -> callback(type, port, name, value, storePos), false));
        }
    }

    // Callback methods which place new devices in a processing queue which is processed every robot period
    // The WPILib callbacks are notified as part of the device creation. This process ensures that the devices complete their setup process
    // This is especially important for SimDevice's because their initalized callbacks can be notified before their values have been created
    // Queuing also ensures that callbacks (which are usually executed asycronously) are processed syncronously with the rest of the robot code

    // Callback for when a Miscellaneous Device is registered
    private static void callback(String deviceName, int deviceHandle) {
        if(deviceName.startsWith("Talon") || deviceName.startsWith("Victor")) {
            // If a new Talon or Victor has been initalized, attempt to link it to a Webots Motor
            // Create a WebotsMotorForwarder for this motor
            final WebotsMotorForwarder fwdr = new WebotsMotorForwarder(Simulation.robot, deviceName);
            // Register a callback for when the Motor Output changes
            SimUtils.registerValueChangedCallback(new SimDeviceSim(deviceName), "Motor Output",
                // Call the callback function
                (name, handle, readonly, value) -> fwdr.callback(name, value),
                // Initalize with current speed
                true);
        }
        if(deviceName.startsWith("navX")) {
            // If a navX is registered, try to link its SimDevice to the Webots robot
            MockGyro.linkGyro();
        }
    }

    // Callback for when a known device type is registered on a Non-Can port
    private static void callback(String type, int port, String name, HALValue value, int storePos) {
        if(type.equals("PWM")) {
            // If a new PWM device has been initalized, attempt to link it to a Webots Motor
            // Register a speed callback on this device
            callbacks.add(new PWMSim(port).registerSpeedCallback(
                // Call a motor forwarder for a callback
                new WebotsMotorForwarder(Simulation.robot, "PWM_" + port),
                // Initalize with current speed
                true));
        }
        if(storePos > -1) {
            callbacks.get(storePos).close();
            callbacks.set(storePos, null);
        }
    }

    // Func template for callbacks
    private static interface CallbackRegisterFunc<T> {
        CallbackStore registerInitializedCallback(T obj, NotifyCallback callback, boolean initialNotify);
    }

}