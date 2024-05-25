package org.carlmontrobotics.lib199;

import java.util.Arrays;
import java.util.concurrent.ConcurrentHashMap;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class MotorErrors {

    private static final ConcurrentHashMap<Integer, CANSparkBase> temperatureSparks = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<Integer, Integer> sparkTemperatureLimits = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<Integer, Integer> overheatedSparks = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<CANSparkBase, Short> flags = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<CANSparkBase, Short> stickyFlags = new ConcurrentHashMap<>();

    public static final int kOverheatTripCount = 5;

    static {
        Lib199Subsystem.registerAsyncPeriodic(MotorErrors::doReportSparkTemp);
        Lib199Subsystem.registerAsyncPeriodic(MotorErrors::printSparkErrorMessages);
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
        System.err.println("Error: " + error.name() + " occurred while configuring " + vendor + " motor");
        System.err.println("Full stack trace:");
        StackTraceElement[] stack = Thread.currentThread().getStackTrace();
        System.err.println(Arrays.toString(stack));
    }

    public static void checkSparkErrors(CANSparkBase spark) {
        //Purposely obviously impersonal to differentiate from actual computer generated errors
        short faults = spark.getFaults();
        short stickyFaults = spark.getStickyFaults();
        short prevFaults = flags.containsKey(spark) ? flags.get(spark) : 0;
        short prevStickyFaults = stickyFlags.containsKey(spark) ? stickyFlags.get(spark) : 0;

        if (spark.getFaults() != 0 && prevFaults != faults) {
        System.err.println("Whoops, big oopsie : fault error(s) with spark id : " + spark.getDeviceId() + ": [ " + formatFaults(spark) + "], ooF!");
        }
        if (spark.getStickyFaults() != 0 && prevStickyFaults != stickyFaults) {
        System.err.println("Bruh, you did an Error : sticky fault(s) error with spark id : " + spark.getDeviceId() + ": " + formatStickyFaults(spark) + ", Ouch!");
        }
        spark.clearFaults();
        flags.put(spark, faults);
        stickyFlags.put(spark, stickyFaults);
    }

    @Deprecated
    public static void checkSparkMaxErrors(CANSparkMax spark) {
        checkSparkErrors((CANSparkBase)spark);
    }

    private static String formatFaults(CANSparkBase spark) {
        String out = "";
        for(FaultID fault: FaultID.values()) {
            if(spark.getFault(fault)) {
                out += (fault.name() + " ");
            }
        }
        return out;
    }

    private static String formatStickyFaults(CANSparkBase spark) {
        String out = "";
        for(FaultID fault: FaultID.values()) {
            if(spark.getStickyFault(fault)) {
                out += (fault.name() + " ");
            }
        }
        return out;
    }

    @Deprecated
    public static void printSparkMaxErrorMessages() {
        printSparkErrorMessages();
    }

    public static void printSparkErrorMessages() {
        flags.keySet().forEach(MotorErrors::checkSparkErrors);
    }

    public static CANSparkMax createDummySparkMax() {
        return DummySparkMaxAnswer.DUMMY_SPARK_MAX;
    }

    @Deprecated
    public static void reportSparkMaxTemp(CANSparkMax spark, TemperatureLimit temperatureLimit) {
        reportSparkMaxTemp(spark, temperatureLimit.limit);
    }

    public static boolean isSparkMaxOverheated(CANSparkMax spark){
      int id = spark.getDeviceId();
      int motorMaxTemp = sparkTemperatureLimits.get(id);
      return ( spark.getMotorTemperature() >= motorMaxTemp );
    }

    @Deprecated
    public static void reportSparkMaxTemp(CANSparkMax spark, int temperatureLimit) {
        reportSparkTemp((CANSparkBase) spark, temperatureLimit);
    }

    public static void reportSparkTemp(CANSparkBase spark, int temperatureLimit) {
        int id = spark.getDeviceId();
        temperatureSparks.put(id, spark);
        sparkTemperatureLimits.put(id, temperatureLimit);
        overheatedSparks.put(id, 0);
    }

    @Deprecated
    public static void doReportSparkMaxTemp() {
        doReportSparkTemp();
    }

    public static void doReportSparkTemp() {
        temperatureSparks.forEach((port, spark) -> {
            double temp = spark.getMotorTemperature();
            double limit = sparkTemperatureLimits.get(port);
            int numTrips = overheatedSparks.get(port);
            String sparkType = "of unknown type";
            if (spark instanceof CANSparkMax) {
                sparkType = "Max";
            } else if (spark instanceof CANSparkFlex) {
                sparkType = "Flex";
            }
            SmartDashboard.putNumber(String.format("Port %d Spark %s Temp", port, sparkType), temp);

            if(numTrips < kOverheatTripCount) {
                if(temp > limit) {
                    overheatedSparks.put(port, ++numTrips);
                } else {
                    overheatedSparks.put(port, 0);
                }
            }

            // Check if temperature exceeds the setpoint or if the controller has already overheated to prevent other code from resetting the current limit after the controller has cooled
            if(numTrips >= kOverheatTripCount) {
                if(numTrips < kOverheatTripCount + 1) {
                    // Set trip count to kOverheatTripCount + 1 to flag that an error message has already been printed
                    // This prevents the error message from being re-printed every time the periodic method is run
                    overheatedSparks.put(port, kOverheatTripCount + 1);
                    System.err.println("Port " + port + " spark is operating at " + temp + " degrees Celsius! It will be disabled until the robot code is restarted.");
                }
                spark.setSmartCurrentLimit(1);
            }
        });
    }

    private MotorErrors() {}

    @Deprecated
    public static enum TemperatureLimit {
        NEO(70), NEO_550(40);

        public final int limit;

        private TemperatureLimit(int limit) {
            this.limit = limit;
        }
    }

}
