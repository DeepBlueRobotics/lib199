package org.carlmontrobotics.lib199.sim;

import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Rule;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import junit.framework.AssertionFailedError;

@RunWith(Parameterized.class)
public class MockedCANCoderTest {

    @ClassRule
    public static TestRules.InitializeHAL simClassRule = new TestRules.InitializeHAL();
    @Rule
    public TestRules.ResetSimDeviceSimData simTestRule = new TestRules.ResetSimDeviceSimData();

    // Dummy parameter array to make it easy to run this test class repeatedly.
    @Parameterized.Parameters
    public static Object[][] data() {
        return new Object[NUM_REPS][0];
    }

    // Number of times to repeatedly run this test class
    private static final int NUM_REPS = 1;

    @Test
    public void testCountUpdatesPosition() {
        try (CANcoder canCoder = new CANcoder(0)) {
            // Set up a SimDevice for the CANcoder
            new MockedCANCoder(canCoder);

            // Consider the current position to be 0 rotations.
            canCoder.setPosition(0.0);

            double timeoutSec = 0.04;
            double delta = 0.001;

            assertPositionEqualsWithinTime(canCoder, 0.0, timeoutSec, delta);

            // Set the position to 0.42 rotations via the SimDevice interface
            SimDeviceSim canCoderSim = new SimDeviceSim("CANCoder", 0);
            canCoderSim.getDouble("count").set(0.42 * MockedCANCoder.kCANCoderCPR);

            assertPositionEqualsWithinTime(canCoder, 0.42, timeoutSec, delta);
        }
    }

    private Timer timer = new Timer();
    private void assertPositionEqualsWithinTime(CANcoder canCoder, double expected, double timeoutSec, double delta) {
        timer.restart();
        while (timer.get() < timeoutSec) {
            // Wait for the CANCoder to update the position. This appears to happen on a separate thread.
            canCoder.getPosition().waitForUpdate(timeoutSec - timer.get());
            if (Math.abs(canCoder.getPosition().getValueAsDouble() - expected) < delta) {
                return;
            }
        }
        throw new AssertionFailedError("Position further than %g from %g for more than %g secs".formatted(delta, expected, timeoutSec));
    }

}
