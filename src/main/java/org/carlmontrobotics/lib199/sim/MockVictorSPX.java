package org.carlmontrobotics.lib199.sim;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.carlmontrobotics.lib199.ErrorCodeAnswer;
import org.carlmontrobotics.lib199.Mocks;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class MockVictorSPX extends MockPhoenixController {
    public MockVictorSPX(int portPWM) {
        super(portPWM);
        motorPWM = new VictorSP(portPWM);
    }

    public static WPI_VictorSPX createMockVictorSPX(int portPWM) {
        return Mocks.createMock(WPI_VictorSPX.class, new MockVictorSPX(portPWM), new ErrorCodeAnswer(), AutoCloseable.class);
    }
}
