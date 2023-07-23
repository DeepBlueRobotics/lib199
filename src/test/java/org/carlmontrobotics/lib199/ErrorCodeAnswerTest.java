package org.carlmontrobotics.lib199;

import org.junit.Test;

public class ErrorCodeAnswerTest {

    @Test
    public void testResponses() throws Exception {
        // Currently ErrorCodeAnswer is not used since we no longer mock phoenix motors

        // Here is the original test for when we did
        // WPI_TalonSRX talon = Mocks.mock(WPI_TalonSRX.class, new ErrorCodeAnswer());
        
        // // Check that primative types return "null"
        // assertEquals(0, talon.get(), 0.01);

        // // Check that ErrorCode functions return ErrorCode.OK
        // assertEquals(ErrorCode.OK, talon.getLastError());
    }

}
