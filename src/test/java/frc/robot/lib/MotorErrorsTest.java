package frc.robot.lib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;

import org.junit.Test;

import frc.robot.lib.testUtils.ErrStreamTest;

public class MotorErrorsTest extends ErrStreamTest {

    public class SensorFaultSparkMax {
        public short getFaults() {
            return (short)FaultID.kSensorFault.value;
        }

        public boolean getFault(FaultID faultID) {
            return faultID == FaultID.kSensorFault;
        }
    }

    public class StickySensorFaultSparkMax {
        public short getStickyFaults() {
            return (short)FaultID.kSensorFault.value;
        }

        public boolean getStickyFault(FaultID faultID) {
            return faultID == FaultID.kSensorFault;
        }
    }
    @Test
    public void testOkErrors() {
        errStream.reset();
        // Null Status
        MotorErrors.reportError((ErrorCode)null);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportError((CANError)null);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportErrors((ErrorCode)null, null);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportErrors((CANError)null, null);
        assertEquals(0, errStream.toByteArray().length);
        
        // Ok Status
        MotorErrors.reportError(ErrorCode.OK);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportError(CANError.kOk);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportErrors(ErrorCode.OK, ErrorCode.OK);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportErrors(CANError.kOk, CANError.kOk);
        assertEquals(0, errStream.toByteArray().length);
    }

    @Test
    public void testOtherErrors() {
        errStream.reset();
        for(ErrorCode code: ErrorCode.values()) {
            if(code != ErrorCode.OK) {
                MotorErrors.reportError(code);
                assertNotEquals(0, errStream.toByteArray().length);
                errStream.reset();
                MotorErrors.reportErrors(ErrorCode.OK, code);
                assertNotEquals(0, errStream.toByteArray().length);
                errStream.reset();
            }
        }
        for(CANError code: CANError.values()) {
            if(code != CANError.kOk) {
                MotorErrors.reportError(code);
                assertNotEquals(0, errStream.toByteArray().length);
                errStream.reset();
                MotorErrors.reportErrors(CANError.kOk, code);
                assertNotEquals(0, errStream.toByteArray().length);
                errStream.reset();
            }
        }
    }

    @Test
    public void testFaultReporting() {
        CANSparkMax sensorFaultSparkMax = Mocks.createMock(CANSparkMax.class, new SensorFaultSparkMax(), false);
        errStream.reset();
        MotorErrors.checkSparkMaxErrors(sensorFaultSparkMax);
        assertNotEquals(0, errStream.toByteArray().length);
        errStream.reset();
        MotorErrors.checkSparkMaxErrors(sensorFaultSparkMax);
        assertEquals(0, errStream.toByteArray().length);
    }

    @Test
    public void testStickyFaultReporting() {
        CANSparkMax stickySensorFaultSparkMax = Mocks.createMock(CANSparkMax.class, new StickySensorFaultSparkMax(), false);
        errStream.reset();
        MotorErrors.checkSparkMaxErrors(stickySensorFaultSparkMax);
        assertNotEquals(0, errStream.toByteArray().length);
        errStream.reset();
        MotorErrors.checkSparkMaxErrors(stickySensorFaultSparkMax);
        assertEquals(0, errStream.toByteArray().length);
    }

    @Test
    public void testDummySparkMax() {
        DummySparkMaxAnswerTest.assertTestResponses(MotorErrors.createDummySparkMax());
    }

}
