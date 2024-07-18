package org.carlmontrobotics.lib199;

import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
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
    private static boolean connectHALSimWSRequestSent = false;

    private static void ensureRegistered() {
        if(registered) {
            return;
        }
        registered = true;
        registerSimulationPeriodic(() -> {
            if (!connectHALSimWSRequestSent) {
                NetworkTableInstance.getDefault()
                        .getStringTopic("/DeepBlueSim/Coordinator/request")
                        .publish().set("connectHALSimWS");
                connectHALSimWSRequestSent = true;
            }
        });
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
