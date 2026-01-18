package org.carlmontrobotics.lib199;

import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentSkipListMap;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class MotorErrors {

    private static final Map<Integer, SparkBase> temperatureSparks = new ConcurrentSkipListMap<>();
    private static final Map<Integer, Integer> sparkTemperatureLimits = new ConcurrentHashMap<>();
    private static final Map<Integer, Integer> overheatedSparks = new ConcurrentHashMap<>();
    private static final Map<SparkBase, Faults> flags = new ConcurrentSkipListMap<>(
            (spark1, spark2) -> (spark1.getDeviceId() - spark2.getDeviceId()));
    private static final Map<SparkBase, Faults> stickyFlags = new ConcurrentSkipListMap<>(
            (spark1, spark2) -> (spark1.getDeviceId() - spark2.getDeviceId()));

    private static final SparkBaseConfig OVERHEAT_MAX_CONFIG = new SparkMaxConfig().smartCurrentLimit(1);
    private static final SparkBaseConfig OVERHEAT_FLEX_CONFIG = new SparkFlexConfig().smartCurrentLimit(1);


    public static final int kOverheatTripCount = 5;

    static {
        Lib199Subsystem.registerPeriodic(() -> {
            MotorErrors.reportNextNSparkTemps(2);
        });
        Lib199Subsystem.registerPeriodic(() -> {
            MotorErrors.reportNextNSparkErrors(2);
        });
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

    public static void checkSparkErrors(SparkBase spark) {
        //Purposely obviously impersonal to differentiate from actual computer generated errors
        // short faults = spark.getFaults();
        Faults faults = spark.getFaults();
        Faults stickyFaults = spark.getStickyFaults();
        Faults prevFaults = flags.getOrDefault(spark, null);
        Faults prevStickyFaults = stickyFlags.getOrDefault(spark, null);

        if (spark.hasActiveFault() && prevFaults!=null && prevFaults.rawBits != faults.rawBits) {
        System.err.println("Fault Errors! (spark id " + spark.getDeviceId() + "): [" + formatFaults(spark) + "], ooF!");
        }
        if (spark.hasStickyFault() && prevStickyFaults!=null && prevStickyFaults.rawBits != stickyFaults.rawBits) {
        System.err.println("Sticky Faults! (spark id " + spark.getDeviceId() + "): [" + formatStickyFaults(spark) + "], Ouch!");
        }
        spark.clearFaults();
        flags.put(spark, faults);
        stickyFlags.put(spark, stickyFaults);
    }

    private static String formatFaults(Faults f) {
        return "" //i hope this makes you proud of yourself, REVLib
            + (f.can            ? "CAN " : "")
            + (f.escEeprom      ? "Flash ROM " : "")
            + (f.firmware       ? "Firmware " : "")
            + (f.gateDriver     ? "Gate Driver " : "")
            + (f.motorType      ? "Motor Type " : "")
            + (f.other          ? "Other " : "")
            + (f.sensor         ? "Sensor " : "")
            + (f.temperature    ? "Temperature " : "")
        ;
    }

    private static String formatFaults(SparkBase spark) {
        Faults f = spark.getFaults();
        return formatFaults(f);
    }

    private static String formatStickyFaults(SparkBase spark) {
        Faults f = spark.getStickyFaults();
        return formatFaults(f);
    }

    public static void printSparkErrorMessages() {
        flags.keySet().forEach(MotorErrors::checkSparkErrors);
    }

    private static int lastSparkErrorIndexReported = 0;

    static void reportNextNSparkErrors(int n) {
        flags.keySet().stream().skip(lastSparkErrorIndexReported).limit(n)
                .forEach(MotorErrors::checkSparkErrors);
        lastSparkErrorIndexReported = (lastSparkErrorIndexReported + n) % flags.size();
    }

    public static boolean isSparkOverheated(SparkBase spark){
      int id = spark.getDeviceId();
      int motorMaxTemp = sparkTemperatureLimits.get(id);
      return ( spark.getMotorTemperature() >= motorMaxTemp );
    }

    public static void reportSparkTemp(SparkBase spark, int temperatureLimit) {
        int id = spark.getDeviceId();
        temperatureSparks.put(id, spark);
        sparkTemperatureLimits.put(id, temperatureLimit);
        overheatedSparks.put(id, 0);
    }

    public static void doReportSparkTemp() {
        temperatureSparks.forEach(MotorErrors::reportSparkTemp);
    }

    private static int lastSparkTempIndexReported = 0;

    static void reportNextNSparkTemps(int n) {
        temperatureSparks.entrySet().stream().skip(lastSparkTempIndexReported).limit(n)
                .forEach((entry) -> reportSparkTemp(entry.getKey(), entry.getValue()));
        lastSparkTempIndexReported = (lastSparkTempIndexReported + n) % temperatureSparks.size();
    }

    private static void reportSparkTemp(int port, SparkBase spark) {
        double temp = spark.getMotorTemperature();
        double limit = sparkTemperatureLimits.get(port);
        int numTrips = overheatedSparks.get(port);
        String sparkType = "of unknown type";
        if (spark instanceof SparkMax) {
            sparkType = "Max";
        } else if (spark instanceof SparkFlex) {
            sparkType = "Flex";
        }
        SmartDashboard.putNumber(String.format("Port %d Spark %s Temp", port, sparkType), temp);

        if (numTrips < kOverheatTripCount) {
            if (temp > limit) {
                overheatedSparks.put(port, ++numTrips);
            } else {
                overheatedSparks.put(port, 0);
            }
        }

        // Check if temperature exceeds the setpoint or if the controller has already
        // overheated to prevent other code from resetting the current limit after the
        // controller has cooled
        if (numTrips >= kOverheatTripCount) {
            if (numTrips < kOverheatTripCount + 1) {
                // Set trip count to kOverheatTripCount + 1 to flag that an error message has
                // already been printed
                // This prevents the error message from being re-printed every time the periodic
                // method is run
                overheatedSparks.put(port, kOverheatTripCount + 1);
                System.err.println("Port " + port + " spark is operating at " + temp
                        + " degrees Celsius! It will be disabled until the robot code is restarted.");
            }
            switch(MotorControllerFactory.getControllerType(spark)){
                case SPARK_MAX:
                    spark.configure(
                        OVERHEAT_MAX_CONFIG,
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters);
                    break;
                case SPARK_FLEX:
                    spark.configure(
                        OVERHEAT_FLEX_CONFIG,
                        ResetMode.kNoResetSafeParameters,
                        PersistMode.kNoPersistParameters);
                    break;
                default:
                    System.err.println("Unknown spark :(");
            }
        }
    }

    private MotorErrors() {}
}
