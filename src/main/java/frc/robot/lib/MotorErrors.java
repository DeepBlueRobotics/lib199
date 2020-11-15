package frc.robot.lib;

import java.util.Arrays;
import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;

import org.mockito.Mockito;

public final class MotorErrors {

    private static final HashMap<CANSparkMax, Short> flags = new HashMap<>();
    private static final HashMap<CANSparkMax, Short> stickyFlags = new HashMap<>();

    public static void reportError(ErrorCode error) {
        reportError("CTRE", error, ErrorCode.OK);
    }

    public static void reportError(CANError error) {
        reportError("REV Robotics", error, CANError.kOk);
    }
    
    public static void reportErrors(ErrorCode... errors) {
        for(ErrorCode error: errors) {
            reportError(error);
        }
    }

    public static void reportErrors(CANError... errors) {
        for(CANError error: errors) {
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
        return Mockito.mock(CANSparkMax.class, new DummySparkMaxAnswer());
    }

    private MotorErrors() {}

}