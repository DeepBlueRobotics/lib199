package org.carlmontrobotics.lib199.testUtils;

import org.carlmontrobotics.lib199.Lib199Subsystem;
import org.junit.rules.TestRule;
import org.junit.runner.Description;
import org.junit.runners.model.Statement;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class TestRules {

    public static class InitializeHAL implements TestRule {
        @Override
        public Statement apply(Statement base, Description description) {
            return new Statement(){
                @Override
                public void evaluate() throws Throwable {
                    // HAL must be initialized or SimDeviceSim.resetData() will crash and SmartDashboard might not work.
                    HAL.initialize(500, 0);
                    base.evaluate();
                    HAL.shutdown();
                }
            };
        }
    }

    public static class ResetSimDeviceSimData implements TestRule {
        @Override
        public Statement apply(Statement base, Description description) {
            return new Statement(){
                @Override
                public void evaluate() throws Throwable {
                    // Ensure there are no async periodic things running before
                    // we reset the SimDeviceData so that they don't try to
                    // touch devices/data after they have been removed.
                    Lib199Subsystem.unregisterAllAsync();
                    SimDeviceSim.resetData();
                    base.evaluate();
                    Lib199Subsystem.unregisterAllAsync();
                    SimDeviceSim.resetData();
                }
            };
        }
    }

}
