package frc.robot.lib.sim;

import java.util.concurrent.CopyOnWriteArrayList;

import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Manages control over the robot simulation and Webots connection
 */
public final class Simulation {

    /**
     * An object representing the Webots robot
     */
    public static final Robot robot;
    /**
     * The value of the basicTimeStep field of the WorldInfo node of the Webots robot
     * @see Robot#getBasicTimeStep()
     */
    public static final int timeStep;
    /**
     * {@link #timeStep} converted into milliseconds. This is equivalent to <code>timeStep * 1000</code>
     */
    public static final double timeStepMillis;
    // Use a CopyOnWriteArrayList to prevent syncronization errors
    private static final CopyOnWriteArrayList<Runnable> periodicMethods;

    static {
        if(RobotBase.isSimulation()) {
            // Initialize Robot
            periodicMethods = new CopyOnWriteArrayList<>();
            robot = new com.cyberbotics.webots.controller.Robot();
            // Make sure to remove the robot when the WPIlib simulation ends
            Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));
            timeStep = (int) Math.round(robot.getBasicTimeStep());
            timeStepMillis = robot.getBasicTimeStep() / 1000;
            // Link to WPILib periodic loop
            Thread updater = new Thread(() -> {
                while(true) {
                    // Sync With Webots
                    robot.step(timeStep);
                    // Run Simulation Periodic Methods
                    periodicMethods.forEach(Runnable::run);
                }
            });
            updater.setDaemon(true);
            updater.start();
            // Register callbacks
            SimRegisterer.init();
        } else {
            robot = null;
            timeStep = 0;
            timeStepMillis = 0;
            periodicMethods = null;
        }
    }

    /**
     * Registers a method to be run as part of the robot's periodic loop
     * @param method
     */
    public static void registerPeriodicMethod(Runnable method) {
        periodicMethods.add(method);
    }

    /**
     * Connects to the Webots robot and initializes the simulation code if the robot is being simulated.
     * This is a convenience method to ensure the static block of this class is run.
     */
    public static void startSimulation() {/*All processing is done in static block*/}

    private Simulation() {}

}