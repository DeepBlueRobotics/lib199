package frc.robot.lib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assume.assumeTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.Test;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Lib199SubsystemTest {

    @Test
    public void testPeriodic() {
        AtomicInteger counter = new AtomicInteger(0);
        Lib199Subsystem.registerPeriodic(() -> counter.addAndGet(1));
        assertEquals("Periodic method called before CommandScheduler.run", 0, counter.get());
        CommandScheduler.getInstance().run();
        assertEquals("Periodic method called more than once or not at all", 1, counter.get());
    }

    @Test
    public void testSimulationPeriodic() {
        assumeTrue(RobotBase.isSimulation());
        AtomicInteger counter = new AtomicInteger(0);
        Lib199Subsystem.registerPeriodic(() -> counter.addAndGet(1));
        assertEquals("Simulation periodic method called before CommandScheduler.run", 0, counter.get());
        CommandScheduler.getInstance().run();
        assertEquals("Simulation periodic method called more than once or not at all", 1, counter.get());
    }

}
