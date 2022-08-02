package org.carlmontrobotics.lib199.logging;

import java.io.IOException;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import org.apache.commons.csv.CSVPrinter;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles data logging code
 */
final class DataLog {

    private static Object[] dataExportBuffer = null;
    private static ArrayList<String> varIds = new ArrayList<>();
    private static HashMap<String, VarType> types = new HashMap<>();
    private static HashMap<String, Object> data = new HashMap<>();
    private static HashMap<String, Supplier<Object>> dataSuppliers = new HashMap<>();
    private static long refFGATime;
    private static boolean isDisabled = false;

    /**
     * Initializes the data logging code and prints variable ids to the csv file or returns if it has already been initialized
     */
    static void init(LocalDateTime time, long refFGATime) {
        if(!GlobalLogInfo.isInit()) {
            return;
        }
        DataLog.refFGATime = refFGATime;
        registerVarBypassErrors(VarType.DOUBLE, "Seconds Since: " + time.format(GlobalLogInfo.dateTimeFormat), () -> ((double)((RobotController.getFPGATime()-DataLog.refFGATime)/1000))/1000);
        try {
            CSVPrinter printer = GlobalLogInfo.getDataPrinter();
            printer.printRecord(varIds.toArray());
        } catch(IOException e) {
            LogUtils.handleLoggingError(false, "printing csv headers", e);
        }
    }

    /**
     * Registers a variable to be logged whenever {@link #logData()} is called. Must be called before {@link #init()}
     * @param type The {@link VarType} of the variable
     * @param id The id to associate with the variable
     * @param supplier The {@link Supplier} that will be used to load the variable data when {@link #fetchData()} is called
     * @throws IllegalArgumentException If the provided variable id has already been registered
     * @throws IllegalStateException If the data logging code has already been initialized
     */
    static void registerVar(VarType type, String id, Supplier<Object> supplier) throws IllegalArgumentException, IllegalStateException {
        LogUtils.checkNotInit();
        if(varIds.contains(id)) {
            throw new IllegalArgumentException("Variable is already registered");
        }
        varIds.add(id);
        types.put(id, type);
        dataSuppliers.put(id, supplier);
    }

    private static void registerVarBypassErrors(VarType type, String id, Supplier<Object> supplier) {
        if(!varIds.contains(id)) {
            varIds.add(0, id);
        }
        types.put(id, type);
        dataSuppliers.put(id, supplier);
    }

    /**
     * Fetches variable data and then prints it to the csv file and {@link SmartDashboard}
     * @throws IllegalStateException If the data logging code is not initialized
     */
    static void logData() throws IllegalStateException {
        LogUtils.checkInit();
        fetchData();
        if(!isDisabled) {
            TimeLog.startDataLogCycle();
            try {
                CSVPrinter printer = GlobalLogInfo.getDataPrinter();
                printer.printRecord((Object[])exportData());
            } catch(IOException e) {
                LogUtils.handleLoggingError(false, "writing data", e);
            }
            TimeLog.endDataLogCycle();
        }
        putSmartDashboardData();
    }

    /**
     * Fetches data from the {@link Supplier}s and puts it into a {@link HashMap} accessable from {@link #getData()}
     * @throws IllegalStateException If the data logging code is not yet initialized
     */
    static void fetchData() throws IllegalStateException {
        LogUtils.checkInit();
        TimeLog.startDataFetchCycle();
        for(String id: varIds) {
            data.put(id, dataSuppliers.get(id).get());
        }
        TimeLog.endDataFetchCycle();
    }

    private static Object[] exportData() {
        if(dataExportBuffer == null) {
            dataExportBuffer = new Object[varIds.size()];
        }
        for(int i = 0; i < varIds.size(); i++) {
            dataExportBuffer[i] = data.get(varIds.get(i));
        }
        return dataExportBuffer;
    }

    /**
     * Puts the last set of fetched data to {@link SmartDashboard} or fetches new data if none has been fetched yet
     * @throws IllegalStateException If new data has to be fetched and the data logging api has not been initialized
     */
    static void putSmartDashboardData() throws IllegalStateException {
        if(data.size() != varIds.size()) {
            fetchData();
        }
        for(int i = 1; i < varIds.size(); i++) {
            String id = varIds.get(i);
            try {
                switch(types.get(id)) {
                    case BOOLEAN:
                        SmartDashboard.putBoolean(id, (Boolean)data.get(id));
                        break;
                    case INTEGER:
                        SmartDashboard.putNumber(id, (Integer)data.get(id));
                        break;
                    case DOUBLE:
                        SmartDashboard.putNumber(id, (Double)data.get(id));
                        break;
                    case STRING:
                        SmartDashboard.putString(id, (String)data.get(id));
                        break;
                }
            } catch(Exception e) {}
        }
    }

    /**
     * @return An {@link ArrayList} containing the ids of all registered variabless
     */
    static ArrayList<String> getVarIds() {
        return new ArrayList<>(varIds);
    }

    /**
     * @return A {@link HashMap} mapping all registered variable ids to their respective {@link VarType}
     */
    static HashMap<String, VarType> getTypes() {
        return new HashMap<>(types);
    }

    /**
     * @return A {@link HashMap} mapping all registered variable ids to their last fetched value or an empty map if data has not yet been fetched
     */
    static HashMap<String, Object> getData() {
        return new HashMap<>(data);
    }

    /**
     * Flushes the data log
     */
    static void flush() {
        try {
            GlobalLogInfo.getDataPrinter().flush();
        } catch(IOException e) {
            System.err.println("Error flushing data file.");
            System.err.println("Full stack trace:");
            e.printStackTrace(System.err);
        }
    }

    /**
     * @return Whether data logging has been disabled by the user
     * @see #setDisabled(boolean)
     */
    static boolean getDisabled() {
        return isDisabled;
    }

    /**
     * Sets whether data logging should be disabled
     * @param disabled The current disabled state
     * @see #getDisabled()
     */
    static void setDisabled(boolean disabled) {
        isDisabled = disabled;
    }

    private DataLog() {}

}