package org.carlmontrobotics.lib199.sim;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class MockTalonSRXTest extends MockPheonixControllerTest {

    @Override
    protected BaseMotorController createController(int portPWM) {
        return MockTalonSRX.createMockTalonSRX(portPWM);
    }

}
