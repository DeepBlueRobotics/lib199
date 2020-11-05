package frc.robot.lib.sim;

import java.util.ArrayList;

import com.cyberbotics.webots.controller.Robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public final class Simulation {

    public static final Robot robot;
    public static final int timeStep;
    public static final double timeStepMillis;
    private static final ArrayList<Runnable> periodicMethods;

    static {
        if(RobotBase.isSimulation()) {
            periodicMethods = new ArrayList<>();
            robot = new com.cyberbotics.webots.controller.Robot();
            // Make sure to remove the robot when the WPIlib simulation ends
            Runtime.getRuntime().addShutdownHook(new Thread(robot::delete));
            timeStep = (int) Math.round(robot.getBasicTimeStep());
            timeStepMillis = robot.getBasicTimeStep() / 1000;
            new Subsystem() {
                @Override
                public void periodic() {
                    robot.step(timeStep);
                    periodicMethods.forEach(Runnable::run);
                }
            }.register();
            SimRegisterer.init();
            MockGyro.linkGyro();
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