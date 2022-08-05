package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.junit.Test;

public class ErrorCodeAnswerTest {

    @Test
    public void testResponses() throws Exception {
        WPI_TalonSRX talon = Mocks.mock(WPI_TalonSRX.class, new ErrorCodeAnswer());
        
        // Check that primative types return "null"
        assertEquals(0, talon.get(), 0.01);

        // Check that ErrorCode functions return ErrorCode.OK
        assertEquals(ErrorCode.OK, talon.getLastError());
    }

}
