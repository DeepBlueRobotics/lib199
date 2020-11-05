package frc.robot.lib.sim;

import java.util.concurrent.CopyOnWriteArrayList;

import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Simulation {

    public static final Robot robot;
    public static final int timeStep;
    public static final double timeStepMillis;
    //Use a CopyOnWriteArrayList to prevent syncronization errors
    private static final CopyOnWriteArrayList<Runnable> periodicMethods;

    static {
        if(RobotBase.isSimulation()) {
            //Initialize Robot
            periodicMethods = new CopyOnWriteArrayList<>();
            robot = new com.cyberbotics.webots.controller.Robot();
            // Make sure to remove the robot when the WPIlib simulation ends
            Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));
            timeStep = (int) Math.round(robot.getBasicTimeStep());
            timeStepMillis = robot.getBasicTimeStep() / 1000;
            //Link to WPILib periodic loop
            new Subsystem() {
                @Override
                public void periodic() {
                    //Sync With Webots
                    robot.step(timeStep);
                    //Run Simulation Periodic Methods
                    periodicMethods.forEach(Runnable::run);
                }
            }.register();
            //Register callbacks
            SimRegisterer.init();
        } else {
            robot = null;
            timeStep = 0;
            timeStepMillis = 0;
            periodicMethods = null;
        }
    }

    public static void registerPeriodicMethod(Runnable method) {
        periodicMethods.add(method);
    }

    public static void startSimulation() {/*All processing is done in static block*/}

    private Simulation() {}

}