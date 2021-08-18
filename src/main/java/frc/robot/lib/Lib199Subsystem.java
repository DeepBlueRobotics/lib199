package frc.robot.lib;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Lib199Subsystem implements Subsystem {

    private static final Lib199Subsystem INSTANCE = new Lib199Subsystem();
    private static final ArrayList<Runnable> periodicMethods = new ArrayList<>();
    private static final ArrayList<Runnable> periodicSimulationMethods = new ArrayList<>();
    private static final Consumer<Runnable> RUN_RUNNABLE = Runnable::run;

    static {
        ensureRegistered();
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

    public static void simulationPeriodic(Runnable method) {
        periodicSimulationMethods.add(method);
    }

    @Override
    public void periodic() {
        periodicMethods.forEach(RUN_RUNNABLE);
    }

    @Override
    public void simulationPeriodic() {
        periodicSimulationMethods.forEach(RUN_RUNNABLE);
    }

    private Lib199Subsystem() {}

}
