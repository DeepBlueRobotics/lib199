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
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A class that keeps track of safe-mode state and provides functions to access common safe-mode features
 *
 * To the best of my knowledge, all safe-mode features are thread-safe
 */
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

    /**
     * Enables safe-mode
     */
    public static void enable() {
        safeModeStatus.set(true);
    }

    /**
     * Disables safe-mode
     */
    public static void disable() {
        safeModeStatus.set(false);
    }

    /**
     * @return Whether safe-mode is enabled
     */
    public static boolean isEnabled() {
        return safeModeStatus.get();
    }

    //#endregion

    //#region Callbacks

    private static final Set<Runnable> onSafeModeEnabled = createEmptyThreadSafeSet();
    private static final Set<Runnable> onSafeModeDisabled = createEmptyThreadSafeSet();

    // Guaranteed to be thread-safe
    // Not guaranteed to have non-extraneous call
    /**
     * Registers a callback to be called when safe-mode is enabled.
     *
     * A couple notes about the state of the callback function:
     * 1. This function is thread-safe
     * 2. The callback function will be called synchronously with the CommandScheduler (unless you call {@link #updateCallbacks()} from another thread)
     * 3. The callback function may be called multiple times in a row, even if the safe-mode state has not changed
     * 4. The callback function will not be called if safe-mode is not enabled
     * 5. The callback function will be called if safe-mode is enabled unless safe-mode is re-disabled before a periodic update
     * 6. The callback function will not necessarily be invoked based on the current state of safe-mode
     *
     * @param runnable The function to run
     */
    public static void onEnabled(Runnable runnable) {
        onSafeModeEnabled.add(runnable);
    }

    /**
     * Registers a callback to be called when safe-mode is disabled.
     *
     * A couple notes about the state of the callback function:
     * 1. This function is thread-safe
     * 2. The callback function will be called synchronously with the CommandScheduler (unless you call {@link #updateCallbacks()} from another thread)
     * 3. The callback function may be called multiple times in a row, even if the safe-mode state has not changed
     * 4. The callback function will not be called if safe-mode is not disabled
     * 5. The callback function will be called if safe-mode is disabled unless safe-mode is re-enabled before a periodic update
     * 6. The callback function will not necessarily be invoked based on the current state of safe-mode
     *
     * @param runnable The function to run
     */
    public static void onDisabled(Runnable runnable) {
        onSafeModeDisabled.add(runnable);
    }

    /**
     * Calls the callbacks for safe-mode state changes if necessary.
     * It should not be necessary to call this function manually as it is automatically invoked as part of the robot's periodic loop via {@link CommandScheduler}.
     *
     * While this function is thread-safe, if you call it from another thread, the callbacks may not be called synchronously with the {@link CommandScheduler}.
     */
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

    /**
     * Creates a {@link Supplier} that returns a constant value when safe-mode is disabled and a different constant value when safe-mode is enabled.
     *
     * @param <T> The type of the constant values
     * @param normalValue The value to return when safe-mode is disabled
     * @param safeValue The value to return when safe-mode is enabled
     * @return A supplier which selects the appropriate value when called
     */
    public static <T> Supplier<T> constant(T normalValue, T safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    /**
     * Creates a {@link BooleanSupplier} that returns a constant value when safe-mode is disabled and a different constant value when safe-mode is enabled.
     *
     * @param normalValue The value to return when safe-mode is disabled
     * @param safeValue The value to return when safe-mode is enabled
     * @return A supplier which selects the appropriate value when called
     */
    public static BooleanSupplier constant(boolean normalValue, boolean safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    /**
     * Creates a {@link DoubleSupplier} that returns a constant value when safe-mode is disabled and a different constant value when safe-mode is enabled.
     *
     * @param normalValue The value to return when safe-mode is disabled
     * @param safeValue The value to return when safe-mode is enabled
     * @return A supplier which selects the appropriate value when called
     */
    public static DoubleSupplier constant(double normalValue, double safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    /**
     * Creates an {@link IntSupplier} that returns a constant value when safe-mode is disabled and a different constant value when safe-mode is enabled.
     *
     * @param normalValue The value to return when safe-mode is disabled
     * @param safeValue The value to return when safe-mode is enabled
     * @return A supplier which selects the appropriate value when called
     */
    public static IntSupplier constant(int normalValue, int safeValue) {
        return () -> isEnabled() ? safeValue : normalValue;
    }

    /**
     * Creates a {@link LongSupplier} that returns a constant value when safe-mode is disabled and a different constant value when safe-mode is enabled.
     *
     * @param normalValue The value to return when safe-mode is disabled
     * @param safeValue The value to return when safe-mode is enabled
     * @return A supplier which selects the appropriate value when called
     */
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

    /**
     * Creates a {@link GenericHID} which alters its outputs while safe-mode is enabled bassed on an underlying {@link GenericHID} implementation.
     *
     * @param <T> The type of the underlying {@link GenericHID}
     * @param joystick The underlying {@link GenericHID} implementation
     * @return A {@link GenericHID} of the same type and based on the given joystick which alters its outputs while safe-mode is enabled
     */
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

    /**
     * Disables a button on a joystick while safe-mode is enabled.
     *
     * @param joystickPort The port of the joystick
     * @param button The button to disable
     */
    public static void disableButton(int joystickPort, int button) {
        safeDisabledButtons.putIfAbsent(joystickPort, createEmptyThreadSafeSet());
        safeDisabledButtons.get(joystickPort).add(button);
    }

    /**
     * Disables a button on a joystick while safe-mode is enabled.
     *
     * @param joystick The joystick to disable the button on
     * @param button The button to disable
     */
    public static void disableButton(GenericHID joystick, int button) {
        disableButton(joystick.getPort(), button);
    }

    /**
     * Disables an axis on a joystick while safe-mode is enabled.
     *
     * @param joystickPort The port of the joystick
     * @param axis The axis to disable
     */
    public static void disableAxis(int joystickPort, int axis) {
        safeDisabledAxes.putIfAbsent(joystickPort, createEmptyThreadSafeSet());
        safeDisabledAxes.get(joystickPort).add(axis);
    }

    /**
     * Disables an axis on a joystick while safe-mode is enabled.
     *
     * @param joystick The joystick to disable the axis on
     * @param axis The axis to disable
     */
    public static void disableAxis(GenericHID joystick, int axis) {
        disableAxis(joystick.getPort(), axis);
    }

    /**
     * Scales an axis on a joystick while safe-mode is enabled.
     *
     * @param joystickPort The port of the joystick
     * @param axis The axis to scale
     * @param factor The factor to scale the axis by
     */
    public static void scaleAxis(int joystickPort, int axis, double factor) {
        safeScaledAxes.putIfAbsent(joystickPort, new ConcurrentHashMap<>());
        safeScaledAxes.get(joystickPort).put(axis, factor);
    }

    /**
     * Scales an axis on a joystick while safe-mode is enabled.
     *
     * @param joystick The joystick to scale the axis on
     * @param axis The axis to scale
     * @param factor The factor to scale the axis by
     */
    public static void scaleAxis(GenericHID joystick, int axis, double factor) {
        scaleAxis(joystick.getPort(), axis, factor);
    }

    /**
     * Disables a POV state (on POV 0) on a joystick while safe-mode is enabled.
     *
     * This is equivalent to calling {@code disablePOV(joystickPort, 0, angle)}.
     *
     * @param joystickPort The port of the joystick
     * @param angle The angle of the POV to disable
     */
    public static void disablePOV(int joystickPort, int angle) {
        disablePOV(joystickPort, 0, angle);
    }

    /**
     * Disables a POV state (on POV 0) on a joystick while safe-mode is enabled.
     *
     * This is equivalent to calling {@code disablePOV(joystick, 0, angle)}.
     *
     * @param joystick The joystick to disable the POV on
     * @param angle The angle of the POV to disable
     */
    public static void disablePOV(GenericHID joystick, int angle) {
        disablePOV(joystick.getPort(), angle);
    }

    /**
     * Disables a POV state on a joystick while safe-mode is enabled.
     *
     * @param joystickPort The port of the joystick
     * @param pov The POV on the joystick
     * @param angle The angle of the POV to disable
     */
    public static void disablePOV(int joystickPort, int pov, int angle) {
        safeDisabledPOVs.putIfAbsent(joystickPort, new ConcurrentHashMap<>());
        safeDisabledPOVs.get(joystickPort).putIfAbsent(angle, createEmptyThreadSafeSet());
        safeDisabledPOVs.get(joystickPort).get(angle).add(angle);
    }

    /**
     * Disables a POV state on a joystick while safe-mode is enabled.
     *
     * @param joystick The joystick to disable the POV on
     * @param pov The POV on the joystick
     * @param angle The angle of the POV to disable
     */
    public static void disablePOV(GenericHID joystick, int pov, int angle) {
        disablePOV(joystick.getPort(), pov, angle);
    }

    //#endregion

    /**
     * Creates an empty thread-safe set.
     *
     * NOTE: Unlike with maps, there are a few different ways to make sets thread-safe
     * I chose this method of creating synchronized sets based on
     * https://stackoverflow.com/questions/6720396/different-types-of-thread-safe-sets-in-java, and https://docs.oracle.com/javase/tutorial/collections/implementations/set.html
     * This method SHOULD NOT be used outside of safe-mode-related code! (hence why it's not public)
     * Please make your own determination for other areas of the code rather than just copying this.
     *
     * @param <T> The type of the set
     * @return An empty thread-safe set of type {@code T}
     */
    static <T> Set<T> createEmptyThreadSafeSet() {
        return Collections.synchronizedSet(new HashSet<>());
    }

}
