package frc.robot.lib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class MotorErrors {

    private static final HashMap<Integer, CANSparkMax> temperatureSparks = new HashMap<>();
    private static final HashMap<Integer, Integer> sparkTemperatureLimits = new HashMap<>();
    private static final ArrayList<Integer> overheatedSparks = new ArrayList<>();
    private static final HashMap<CANSparkMax, Short> flags = new HashMap<>();
    private static final HashMap<CANSparkMax, Short> stickyFlags = new HashMap<>();

    static {
        Lib199Subsystem.registerPeriodic(MotorErrors::doReportSparkMaxTemp);
    }

    public static void reportError(ErrorCode error) {
        reportError("CTRE", error, ErrorCode.OK);
    }

    public static void reportError(REVLibError error) {
        reportError("REV Robotics", error, REVLibError.kOk);
    }

    public static void reportErrors(ErrorCode... errors) {
        for(ErrorCode error: errors) {
            reportError(error);
        }
    }

    public static void reportErrors(REVLibError... errors) {
        for(REVLibError error: errors) {
            reportError(error);
        }
    }

    private static <T extends Enum<T>> void reportError(String vendor, T error, T ok) {
        if(error == null || error == ok) {
            return;
        }
        System.err.println("Error: " + error.name() + " occured while configuring " + vendor + " motor");
        System.err.println("Full stack trace:");
        StackTraceElement[] stack = Thread.currentThread().getStackTrace();
        System.err.println(Arrays.toString(stack));
    }

    public static void checkSparkMaxErrors(CANSparkMax spark) {
        //Purposely obivously impersonal to differentiate from actual computer generated errors
        short faults = spark.getFaults();
        short stickyFaults = spark.getStickyFaults();
        short prevFaults = flags.containsKey(spark) ? flags.get(spark) : 0;
        short prevStickyFaults = stickyFlags.containsKey(spark) ? stickyFlags.get(spark) : 0;

        if (spark.getFaults() != 0 && prevFaults != faults) {
        System.err.println("Whoops, big oopsie : fault error(s) with spark max id : " + spark.getDeviceId() + ": [ " + formatFaults(spark) + "], ooF!");
        }
        if (spark.getStickyFaults() != 0 && prevStickyFaults != stickyFaults) {
        System.err.println("Bruh, you did an Error : sticky fault(s) error with spark max id : " + spark.getDeviceId() + ": " + formatStickyFaults(spark) + ", Ouch!");
        }
        spark.clearFaults();
        flags.put(spark, faults);
        stickyFlags.put(spark, stickyFaults);
    }

    private static String formatFaults(CANSparkMax spark) {
        String out = "";
        for(FaultID fault: FaultID.values()) {
            if(spark.getFault(fault)) {
                out += (fault.name() + " ");
            }
        }
        return out;
    }

    private static String formatStickyFaults(CANSparkMax spark) {
        String out = "";
        for(FaultID fault: FaultID.values()) {
            if(spark.getStickyFault(fault)) {
                out += (fault.name() + " ");
            }
        }
        return out;
    }

    public static void printSparkMaxErrorMessages() {
        flags.keySet().forEach((spark) -> checkSparkMaxErrors(spark));
    }

    public static CANSparkMax createDummySparkMax() {
        return DummySparkMaxAnswer.DUMMY_SPARK_MAX;
    }

    public static void reportSparkMaxTemp(CANSparkMax spark, TemperatureLimit temperatureLimit) {
        reportSparkMaxTemp(spark, temperatureLimit.limit);
    }

    public static void reportSparkMaxTemp(CANSparkMax spark, int temperatureLimit) {
        int id = spark.getDeviceId();
        temperatureSparks.put(id, spark);
        sparkTemperatureLimits.put(id, temperatureLimit);
    }

    public static void doReportSparkMaxTemp() {
        temperatureSparks.forEach((port, spark) -> {
            double temp = spark.getMotorTemperature();
            SmartDashboard.putNumber("Port " + port + " Spark Max Temp", temp);
            // Check if temperature exceeds the setpoint or if the contoller has already overheated to prevent other code from resetting the current limit after the controller has cooled
            if(temp >= sparkTemperatureLimits.get(port) || overheatedSparks.contains(port)) {
                if(!overheatedSparks.contains(port)) {
                    overheatedSparks.add(port);
                    System.err.println("Port " + port + " spark max is operating at " + temp + " degrees Celsius! It will be disabled until the robot code is restarted.");
                }
                spark.setSmartCurrentLimit(1);
            }
        });
    }

    private MotorErrors() {}

    public static enum TemperatureLimit {
        NEO(70), NEO_550(40);

        public final int limit;

        private TemperatureLimit(int limit) {
            this.limit = limit;
        }
    }

}