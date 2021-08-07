package frc.robot.lib.testUtils;

import org.junit.rules.TestRule;
import org.junit.runner.Description;
import org.junit.runners.model.Statement;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimDeviceTestRule {

    public static class Class implements TestRule {
        @Override
        public Statement apply(Statement base, Description description) {
            return new Statement(){
                @Override
                public void evaluate() throws Throwable {
                    // HAL must be initialized or SimDeviceSim.resetData() will crash
                    HAL.initialize(500, 0);
                    base.evaluate();
                    HAL.shutdown();
                }
            };
        }
    }

    public static class Test implements TestRule {
        @Override
        public Statement apply(Statement base, Description description) {
            return new Statement(){
                @Override
                public void evaluate() throws Throwable {
                    SimDeviceSim.resetData();
                    base.evaluate();
                    SimDeviceSim.resetData();
                }
            };
        }
    }

}
