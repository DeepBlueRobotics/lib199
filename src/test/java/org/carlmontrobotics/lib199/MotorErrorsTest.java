package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;

import org.carlmontrobotics.lib199.testUtils.ErrStreamTest;
import org.junit.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class MotorErrorsTest extends ErrStreamTest {

    public static class SensorFaultSparkMax {
        public short getFaults() {
            return (short)FaultID.kSensorFault.value;
        }

        public boolean getFault(FaultID faultID) {
            return faultID == FaultID.kSensorFault;
        }
    }

    public static class StickySensorFaultSparkMax {
        public short getStickyFaults() {
            return (short)FaultID.kSensorFault.value;
        }

        public boolean getStickyFault(FaultID faultID) {
            return faultID == FaultID.kSensorFault;
        }
    }

    public static interface TemperatureSparkMax {

        public void setTemperature(double temperature);
        public int getSmartCurrentLimit();
        public double getMotorTemperature();
        public REVLibError setSmartCurrentLimit(int limit);

        public static class Instance {

            private int smartCurrentLimit = MotorConfig.NEO.currentLimit;
            private double temperature = 30;
            private final int id;

            public Instance(int id) {
                this.id = id;
            }

            public void setTemperature(double temperature) {
                this.temperature = temperature;
            }

            public int getSmartCurrentLimit() {
                return smartCurrentLimit;
            }

            public double getMotorTemperature() {
	        	return temperature;
            }

            public REVLibError setSmartCurrentLimit(int limit) {
                smartCurrentLimit = limit;
    		    return REVLibError.kOk;
            }
            
            public int getDeviceId() {
                return id;
            }
        }

    }

    @Test
    public void testOkErrors() {
        errStream.reset();
        // Null Status
        MotorErrors.reportError((ErrorCode)null);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportError((REVLibError)null);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportErrors((ErrorCode)null, null);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportErrors((REVLibError)null, null);
        assertEquals(0, errStream.toByteArray().length);
        
        // Ok Status
        MotorErrors.reportError(ErrorCode.OK);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportError(REVLibError.kOk);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportErrors(ErrorCode.OK, ErrorCode.OK);
        assertEquals(0, errStream.toByteArray().length);
        MotorErrors.reportErrors(REVLibError.kOk, REVLibError.kOk);
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
        for(REVLibError code: REVLibError.values()) {
            if(code != REVLibError.kOk) {
                MotorErrors.reportError(code);
                assertNotEquals(0, errStream.toByteArray().length);
                errStream.reset();
                MotorErrors.reportErrors(REVLibError.kOk, code);
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

    @Test
    public void testReportSparkMaxTemp() {
        assertTrue(MotorErrors.kOverheatTripCount > 0);

        doTestReportSparkMaxTemp(0);
        doTestReportSparkMaxTemp(1);
        doTestReportSparkMaxTemp(2);
    }

    private void doTestReportSparkMaxTemp(int id) {
        TemperatureSparkMax spark = (TemperatureSparkMax)Mocks.createMock(CANSparkMax.class, new TemperatureSparkMax.Instance(id), TemperatureSparkMax.class);
        String smartDashboardKey = "Port " + id + " Spark Max Temp";
        MotorErrors.reportSparkMaxTemp((CANSparkMax)spark, 40);
        spark.setSmartCurrentLimit(50);

        spark.setTemperature(20);
        CommandScheduler.getInstance().run();
        assertEquals(20, SmartDashboard.getNumber(smartDashboardKey, 0), 0.01);
        assertEquals(50, spark.getSmartCurrentLimit());

        spark.setTemperature(20);
        CommandScheduler.getInstance().run();
        assertEquals(20, SmartDashboard.getNumber(smartDashboardKey, 0), 0.01);
        assertEquals(50, spark.getSmartCurrentLimit());

        if(MotorErrors.kOverheatTripCount > 1) {
            spark.setTemperature(51);
            CommandScheduler.getInstance().run();
            assertEquals(51, SmartDashboard.getNumber(smartDashboardKey, 0), 0.01);
            assertEquals(50, spark.getSmartCurrentLimit());

            spark.setTemperature(20);
            CommandScheduler.getInstance().run();
            assertEquals(20, SmartDashboard.getNumber(smartDashboardKey, 0), 0.01);
            assertEquals(50, spark.getSmartCurrentLimit());
        }

        assertEquals(0, errStream.size());

        for(int i = 0; i < MotorErrors.kOverheatTripCount; i++) {
            assertEquals(50, spark.getSmartCurrentLimit());
            assertEquals(0, errStream.size());

            spark.setTemperature(51);
            CommandScheduler.getInstance().run();
            assertEquals(51, SmartDashboard.getNumber(smartDashboardKey, 0), 0.01);
        }

        assertNotEquals(0, errStream.size());
        errStream.reset();

        spark.setTemperature(51);
        CommandScheduler.getInstance().run();
        assertEquals(51, SmartDashboard.getNumber(smartDashboardKey, 0), 0.01);
        assertEquals(1, spark.getSmartCurrentLimit());

        spark.setTemperature(20);
        CommandScheduler.getInstance().run();
        assertEquals(20, SmartDashboard.getNumber(smartDashboardKey, 0), 0.01);
        assertEquals(1, spark.getSmartCurrentLimit());

        assertEquals(0, errStream.size());
    }

}
