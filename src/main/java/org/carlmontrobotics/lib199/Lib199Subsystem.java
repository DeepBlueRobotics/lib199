package org.carlmontrobotics.lib199;

import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Lib199Subsystem implements Subsystem {

    private static final Lib199Subsystem INSTANCE = new Lib199Subsystem();
    private static final CopyOnWriteArrayList<Runnable> periodicMethods = new CopyOnWriteArrayList<>();
    private static final CopyOnWriteArrayList<Runnable> periodicSimulationMethods = new CopyOnWriteArrayList<>();
    private static final CopyOnWriteArrayList<Runnable> asyncPeriodicMethods = new CopyOnWriteArrayList<>();
    private static final CopyOnWriteArrayList<Runnable> asyncPeriodicSimulationMethods = new CopyOnWriteArrayList<>();
    private static final Consumer<Runnable> RUN_RUNNABLE = Runnable::run;

    private static final Thread asyncPeriodicThread;

    public static final long asyncSleepTime = 20;

    static {
        ensureRegistered();

        asyncPeriodicThread = new Thread(() -> {
            while(true) {
                INSTANCE.asyncPeriodic();
                try {
                    Thread.sleep(asyncSleepTime);
                } catch(InterruptedException e) {}
            }
        });
        asyncPeriodicThread.setDaemon(true);
        asyncPeriodicThread.start();
    }

    private static boolean registered = false;

    private static void ensureRegistered() {
        if(registered) {
            return;
        }
        registered = true;
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

    public static void registerAsyncPeriodic(Runnable method) {
        asyncPeriodicMethods.add(method);
    }

    public static void registerAsyncSimulationPeriodic(Runnable method) {
        if(RobotBase.isSimulation()) asyncPeriodicSimulationMethods.add(method);
    }

    @Override
    public void periodic() {
        periodicMethods.forEach(RUN_RUNNABLE);
    }

    @Override
    public void simulationPeriodic() {
        periodicSimulationMethods.forEach(RUN_RUNNABLE);
    }

    public void asyncPeriodic() {
        asyncPeriodicMethods.forEach(RUN_RUNNABLE);
        asyncPeriodicSimulationMethods.forEach(RUN_RUNNABLE);
    }

    private Lib199Subsystem() {}

}
