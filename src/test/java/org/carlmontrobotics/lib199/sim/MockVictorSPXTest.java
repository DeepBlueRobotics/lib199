package org.carlmontrobotics.lib199.sim;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class MockVictorSPXTest extends MockPheonixControllerTest {

    @Override
    protected BaseMotorController createController(int portPWM) {
        return MockVictorSPX.createMockVictorSPX(portPWM);
    }
    
}
