package frc.robot.lib.logging;

/**
 * Represents the type of a variable in the logging code
 * @deprecated Instead use <a href="https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html">WPILib's Logging API</a>
 */
@Deprecated
public enum VarType {
    BOOLEAN, INTEGER, DOUBLE, STRING;
}