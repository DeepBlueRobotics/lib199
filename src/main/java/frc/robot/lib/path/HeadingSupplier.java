package frc.robot.lib.path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public abstract class HeadingSupplier {
    private Timer timer;
    private boolean timerStarted;

    /**
     * Constructs a HeadingSupplier object
     */
    public HeadingSupplier() {
        timer = new Timer();
        timerStarted = false;
    }

    /**
     * Gets the desired rotation at current point in time
     * 
     * @return current trajectory rotation at current point in time
     */
    public Rotation2d sample() {            
        if (!timerStarted) {
            timerStarted = true;
            timer.start();
        }
        return getSample(timer.get());
    }

    protected abstract Rotation2d getSample(double currentTime);

    /**
     * Reset the HeadingSupplier
     */
    public void reset() {
        timerStarted = false;
        timer.reset();
    }
}
