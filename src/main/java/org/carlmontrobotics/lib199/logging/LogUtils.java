package org.carlmontrobotics.lib199.logging;

import static org.carlmontrobotics.lib199.logging.GlobalLogInfo.*;

/**
 * Various utility methods utilized by the logging code
 * @deprecated Instead use <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html">WPILib's Logging API</a>
 */
@Deprecated
final class LogUtils {

    /**
     * Checks that the logging api is initialized
     * @throws IllegalStateException If the logging code is not initialized
     * @see #checkNotInit()
     */
    static void checkInit() throws IllegalStateException {
        if(!isInit()) {
            throw new IllegalStateException("Logging code is not initialized");
        }
    }

    /**
     * Checks that the logging api is not initialized
     * @throws IllegalStateException If the logging code is already initialized
     * @see #checkInit()
     */
    static void checkNotInit() throws IllegalStateException {
        if(GlobalLogInfo.isInit()) {
            throw new IllegalStateException("Logging code already initialized");
        }
    }

    /**
     * Handles an {@link Exception} in the logging code by notifying the user and disabling the corresponding sector of code
     * @param sector The sector to disable. <code>true</code> for event logging, and <code>false</code> for data logging
     * @param task A string representing the task that caused the error to occur
     * @param error The {@link Exception} that occured or <code>null</code> if the exception could not be obtained
     * @see #handleLoggingApiDisableError(String, Exception)
     */
    static void handleLoggingError(boolean sector, String task, Exception error) {
        if(sector) {
            System.err.println("Error occured while " + task + ". Logging will continue to run with event logging disabled.");
            if(error != null) {
                System.err.println("Full stack trace:");
                error.printStackTrace(System.err);
            }
            disableEvents();
        } else {
            System.err.println("Error occured while " + task + " logging will continue to run with data logging disabled.");
            if(error != null) {
                System.err.println("Full stack trace:");
                error.printStackTrace(System.err);
            }
            disableData();
        }
    }

    /**
     * Handles an {@link Exception} in the logging code that causes the failure of all of the logging code by notifying the user and disabling the logging code
     * @param task A string representing the task that caused the error to occur
     * @param error The {@link Exception} that occured or <code>null</code> if the exception could not be obtained
     * @see #handleLoggingError(boolean, String, Exception)
     */
    static void handleLoggingApiDisableError(String task, Exception error) {
        System.err.println("Error occured while " + task + ". Logging will be disabled.");
        if(error != null) {
            System.err.println("Full stack trace:");
            error.printStackTrace(System.err);
        }
        disableEvents();
        disableData();
    }

    /**
     * Handles an {@link Exception} caused by an illegal operation by the user by notifying the user
     * @param e The {@link Exception} to handle
     */
	static void handleException(Exception e) {
        System.err.print("An Exception occurred in logging code");
        if(e == null) {
            System.err.println(". No relevent information regarding the error could be obtained. "
                + "IsInit=" + GlobalLogInfo.isInit() + " " + getStateMessage());
                return;
        }
        StackTraceElement thrower = e.getStackTrace()[0];
        StackTraceElement caller = findCaller(e.getStackTrace());
        System.err.print(": " + e.getClass().getName() + ": " + e.getMessage() + ". The error originated in: " + formatElement(thrower) + ". ");
        if(caller == null) {
            System.err.print("No infornmation could be obtained about the caller that caused this error.");
        } else {
            System.err.print("Which was called by: " + formatElement(caller) + ". ");
        }
        System.err.println(getStateMessage());
        System.err.println("Full stack trace:");
        e.printStackTrace(System.err);
    }
    
    private static StackTraceElement findCaller(StackTraceElement[] stack) {
        for(StackTraceElement e: stack) {
            if(!e.getClassName().substring(0, e.getClassName().lastIndexOf(".") == -1 ? 0 : e.getClassName().lastIndexOf(".")).equals(LogUtils.class.getPackageName())) {
                return e;
            }
        }
        return null;
    }

    private static String formatElement(StackTraceElement e) {
        return e.getClassName() + " on line: " + e.getLineNumber();
    }

    private static String getStateMessage() {
        if(isInit()) {
            return "You should try moving the offending method to before Log.init(). ";
        } else {
            
            return "You should try moving the offending method to after Log.init(). ";
        }
    }

    private LogUtils() {}

}