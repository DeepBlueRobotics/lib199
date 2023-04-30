package org.carlmontrobotics.lib199.safeMode;

import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import org.carlmontrobotics.lib199.Lib199Subsystem;
import org.carlmontrobotics.lib199.Mocks;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SafeMode {

    //#region Basic Safe Mode

    private static BooleanEntry safeModeStatus;

    // NetworkTables doesn't like it when we use the same subscriber for isEnabled() and updateCallbacks()
    // My guess is that readQueueValues() interferes with the get() call
    private static BooleanSubscriber enabledListener;

    static {
        SmartDashboard.putBoolean("Safe Mode", false);
        BooleanTopic safeModeTopic = new BooleanTopic(SmartDashboard.getEntry("Safe Mode").getTopic());
        safeModeStatus = safeModeTopic.getEntry(false);
        enabledListener = safeModeTopic.subscribe(false);

        // Call updateCallbacks synchronously with the CommandScheduler
        Lib199Subsystem.registerPeriodic(SafeMode::updateCallbacks);
    }

    public static void enable() {
        safeModeStatus.set(true);
    }

    public static void disable() {
        safeModeStatus.set(false);
    }

    public static boolean isEnabled() {
        return safeModeStatus.get();
    }

    //#endregion

    //#region Callbacks

    private static final Set<Runnable> onSafeModeEnabled = createEmptyThreadSafeSet();
    private static final Set<Runnable> onSafeModeDisabled = createEmptyThreadSafeSet();

    // Guaranteed to be thread-safe
    // Not guaranteed to have non-extraneous call
    public static void onEnabled(Runnable runnable) {
        onSafeModeEnabled.add(runnable);
    }

    // Guaranteed to be thread-safe
    // Not guaranteed to have non-extraneous call
    public static void onDisabled(Runnable runnable) {
        onSafeModeDisabled.add(runnable);
    }

    public static void updateCallbacks() {
        boolean stateChanged = enabledListener.readQueueValues().length != 0;
        if(stateChanged) {
            if (isEnabled()) {
                onSafeModeEnabled.forEach(Runnable::run);
            } else {
                onSafeModeDisabled.forEach(Runnable::run);
            }
        }
    }

    //#endregion

    //#region Safe Constants

    public static <T> Supplier<T> constant(T normalValue, T safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    public static BooleanSupplier constant(boolean normalValue, boolean safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    public static DoubleSupplier constant(double normalValue, double safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    public static IntSupplier constant(int normalValue, int safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    public static LongSupplier constant(long normalValue, long safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    //#endregion

    //#region Safe Joystick

    // lib199 uses asynchronous code in a few places, so these will all be thread-safe
    private static final Map<Integer, Set<Integer>> safeDisabledButtons = new ConcurrentHashMap<>();
    private static final Map<Integer, Set<Integer>> safeDisabledAxes = new ConcurrentHashMap<>();
    private static final Map<Integer, Map<Integer, Double>> safeScaledAxes = new ConcurrentHashMap<>();
    private static final Map<Integer, Map<Integer, Set<Integer>>> safeDisabledPOVs = new ConcurrentHashMap<>();

    @SuppressWarnings("unchecked")
    public static <T extends GenericHID> T makeSafe(T joystick) {
        safeDisabledAxes.putIfAbsent(joystick.getPort(), createEmptyThreadSafeSet());
        safeDisabledButtons.putIfAbsent(joystick.getPort(), createEmptyThreadSafeSet());
        safeScaledAxes.putIfAbsent(joystick.getPort(), new ConcurrentHashMap<>());
        safeDisabledPOVs.putIfAbsent(joystick.getPort(), new ConcurrentHashMap<>());

        int port = joystick.getPort();
        return Mocks.createMock(
            (Class<T>) joystick.getClass(),
            new SafeJoystick(
                joystick,
                safeDisabledButtons.get(port),
                safeDisabledAxes.get(port),
                safeScaledAxes.get(port),
                safeDisabledPOVs.get(port)
            )
        );
    }

    public static void disableButton(int joystickPort, int button) {
        safeDisabledButtons.putIfAbsent(joystickPort, createEmptyThreadSafeSet());
        safeDisabledButtons.get(joystickPort).add(button);
    }

    public static void disableAxis(int joystickPort, int axis) {
        safeDisabledAxes.putIfAbsent(joystickPort, createEmptyThreadSafeSet());
        safeDisabledAxes.get(joystickPort).add(axis);
    }

    public static void scaleAxis(int joystickPort, int axis, double factor) {
        safeScaledAxes.putIfAbsent(joystickPort, new ConcurrentHashMap<>());
        safeScaledAxes.get(joystickPort).put(axis, factor);
    }

    public static void disablePOV(int joystickPort, int angle) {
        disablePOV(joystickPort, 0, angle);
    }

    public static void disablePOV(int joystickPort, int pov, int angle) {
        safeDisabledPOVs.putIfAbsent(joystickPort, new ConcurrentHashMap<>());
        safeDisabledPOVs.get(joystickPort).putIfAbsent(angle, createEmptyThreadSafeSet());
        safeDisabledPOVs.get(joystickPort).get(angle).add(angle);
    }

    //#endregion

    // NOTE: Unlike with maps, there are a few different ways to make sets thread-safe
    // I chose this method of creating synchronized sets based on
    // https://stackoverflow.com/questions/6720396/different-types-of-thread-safe-sets-in-java, and https://docs.oracle.com/javase/tutorial/collections/implementations/set.html
    // Please make your own determination for other areas of the code rather than just copying this
    static <T> Set<T> createEmptyThreadSafeSet() {
        return Collections.synchronizedSet(new HashSet<>());
    }

}
