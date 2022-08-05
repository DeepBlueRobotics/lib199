package org.carlmontrobotics.lib199.sim;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.carlmontrobotics.lib199.ErrorCodeAnswer;
import org.carlmontrobotics.lib199.Mocks;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class MockTalonSRX extends MockPhoenixController {

    public MockTalonSRX(int portPWM) {
        super(portPWM);
        motorPWM = new Talon(portPWM);
    }

    public static WPI_TalonSRX createMockTalonSRX(int portPWM) {
        return Mocks.createMock(WPI_TalonSRX.class, new MockTalonSRX(portPWM), new ErrorCodeAnswer(), AutoCloseable.class);
    }
}