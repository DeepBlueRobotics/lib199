package org.carlmontrobotics.lib199.path;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

//Findd Me
public class PPRobotPath {

    private PathPlannerTrajectory trajectory;
    private SwerveDriveInterface dt;
    private HashMap<String, Command> eventMap;

    /**
     * Constructs a RobotPath Object
     * 
     * @param pathName   Name of the path
     * @param dt         Drivetrain object
     * @param reversed   Whether the path should be reversed
     * @param eventMap   Map of event marker names to the commands that should run when reaching that marker.
     *                   This SHOULD NOT contain any commands requiring the same subsystems as this command, or it will be interrupted
     */
    public PPRobotPath(String pathName, SwerveDriveInterface dt, boolean reversed, HashMap<String, Command> eventMap) {
        this(
            PathPlanner.loadPath(pathName, dt.getMaxSpeedMps(), dt.getMaxAccelMps2(), reversed),
            dt,
            eventMap
        );
    }

    /**
     * Constructs a RobotPath Object
     * 
     * @param trajectory The trajectory to follow
     * @param dt         Drivetrain object
     * @param eventMap   Map of event marker names to the commands that should run when reaching that marker.
     *                   This SHOULD NOT contain any commands requiring the same subsystems as this command, or it will be interrupted
     */
    public PPRobotPath(PathPlannerTrajectory trajectory, SwerveDriveInterface dt, HashMap<String, Command> eventMap) {
        this.trajectory = trajectory;
        this.dt = dt;
        this.eventMap = eventMap;
    }

    /**
     * Constructs a RobotPath Object
     * 
     * @param initializeDrivetrainPosition
     * @param stopAtEnd
     * @return
     */
    public PPRobotPath(String pathName, SwerveDriveInterface dt, double maxVel, double maxAccel, boolean reversed, HashMap<String, Command> eventMap) {
        this.trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel, reversed);
        this.dt = dt;
        this.eventMap = eventMap;
    }

    /**
     * Gets a path command for the given path
     * 
     * @param initializeDrivetrainPosition whether the robot should assume it is currently at the start of the path
     * @param stopAtEnd                    whether the robot should stop at the end
     * @return PathCommand
     */
    public Command getPathCommand(boolean initializeDrivetrainPosition, boolean stopAtEnd) {
        Command command = dt.createPPAutoCommand(trajectory, eventMap);
        if(initializeDrivetrainPosition) {
            command = new InstantCommand(this::initializeDrivetrainPosition).andThen(command);
        }
        if (stopAtEnd) {
            command = command.andThen(new InstantCommand(dt::stop, dt));
        }
        return command;
    }

    /**
     * Tells the drivetrain to assume that the robot is at the starting position of
     * this path.
     */
    public void initializeDrivetrainPosition() {
        dt.setPose(getInitialPose());
    }

    /**
     * Gets the initial pose of the path
     *
     * @return Initial pose
     */
    public Pose2d getInitialPose() {
        PathPlannerState state = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance()).getInitialState();
        return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
    }

    /**
     * Adds an event to the event map
     * @param event    The name of the event
     * @param command  The command to run
     *                 This SHOULD NOT contain any commands requiring Drivetrain, or it will be interrupted
     */
    public void addEvent(String event, Command command) {
        eventMap.put(event, command);
    }

    /**
     * Adds multiple events to the event map
     * @param events The events to add
     */
    public void addEvent(Map<String, Command> events) {
        eventMap.putAll(events);
    }
}
