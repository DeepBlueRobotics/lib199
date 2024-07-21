package org.carlmontrobotics.lib199;

import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Lib199Subsystem implements Subsystem {

    private static final Lib199Subsystem INSTANCE = new Lib199Subsystem();
    private static final CopyOnWriteArrayList<Runnable> periodicMethods = new CopyOnWriteArrayList<>();
    private static final CopyOnWriteArrayList<Runnable> periodicSimulationMethods = new CopyOnWriteArrayList<>();
    private static final Consumer<Runnable> RUN_RUNNABLE = Runnable::run;

    @Deprecated
    public static final long asyncSleepTime = 20;

    static {
        ensureRegistered();
    }

    private static boolean registered = false;

    private static void ensureRegistered() {
        if(registered) {
            return;
        }
        registered = true;

        // Regularly request a HALSimWS connection from the DeepBlueSim controller (if/when it is
        // listening). To workaround https://github.com/wpilibsuite/allwpilib/issues/6842, this must
        // be done *after* any SimDevices have been created (like those used to support simulation
        // of many of the devices in lib199).
        var reqPublisher = NetworkTableInstance.getDefault()
                .getStringTopic("/DeepBlueSim/Coordinator/request").publish();
        registerSimulationPeriodic(() -> reqPublisher.set("connectHALSimWS"));

        INSTANCE.register();
    }

    public static void registerPeriodic(Runnable method) {
        periodicMethods.add(method);
    }

    @Deprecated
    /**
     * @deprecated Use registerSimulationPeriodic
     * @param method
     */
    public static void simulationPeriodic(Runnable method) {
        registerSimulationPeriodic(method);
    }

    public static void registerSimulationPeriodic(Runnable method) {
        periodicSimulationMethods.add(method);
    }

    @Deprecated
    /**
     * @deprecated Use registerPeriodic
     * @param method
     */
    public static void registerAsyncPeriodic(Runnable method) {
        registerPeriodic(method);
    }

    @Deprecated
    /**
     * @deprecated Use registerSimulationPeriodic
     * @param method
     */
    public static void registerAsyncSimulationPeriodic(Runnable method) {
        registerSimulationPeriodic(method);
    }

    @Override
    public void periodic() {
        periodicMethods.forEach(RUN_RUNNABLE);
    }

    @Override
    public void simulationPeriodic() {
        periodicSimulationMethods.forEach(RUN_RUNNABLE);
    }

    @Deprecated
    /**
     * No longer does anything.
     */
    public synchronized void asyncPeriodic() {
    }

    private Lib199Subsystem() {}

}
