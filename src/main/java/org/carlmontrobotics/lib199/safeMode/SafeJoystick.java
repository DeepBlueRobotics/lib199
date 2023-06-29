package org.carlmontrobotics.lib199.safeMode;

import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A wrapper for a {@link GenericHID} that implements safe-mode features.
 *
 * This class is designed for internal use via {@link org.carlmontrobotics.lib199.Mocks}.
 * As such it provides re-implementations for {@link GenericHID} methods without extending {@link GenericHID}
 * to allow for the extension of subclasses such as {@link edu.wpi.first.wpilibj.Joystick}, {@link edu.wpi.first.wpilibj.PS4Controller},
 * {@link edu.wpi.first.wpilibj.XboxController}, etc. To wrap a joystick, call {@link SafeMode#makeSafe(GenericHID)}.
 */
public class SafeJoystick {

    /**
     * The unsafe joystick that this class wraps.
     */
    public final GenericHID unsafeJoystick;

    private final Set<Integer> safeDisabledButtons;
    private final Set<Integer> safeDisabledAxes;
    private final Map<Integer, Double> safeScaledAxes;
    private final Map<Integer, Set<Integer>> safeDisabledPOV;

    /**
     * Creates a new SafeJoystick
     *
     * @param unsafeJoystick The unsafe joystick to wrap
     * @param safeDisabledButtons The buttons that should be disabled in safe-mode
     * @param safeDisabledAxes The axes that should be disabled in safe-mode
     * @param safeScaledAxes The axes that should be scaled in safe-mode (key: axis, value: scale factor)
     * @param safeDisabledPOV The POVs that should be disabled in safe-mode (key: POV, value: set of disabled values)
     */
    public SafeJoystick(GenericHID unsafeJoystick, Set<Integer> safeDisabledButtons, Set<Integer> safeDisabledAxes, Map<Integer, Double> safeScaledAxes, Map<Integer, Set<Integer>> safeDisabledPOV) {
        this.unsafeJoystick = unsafeJoystick;
        this.safeDisabledButtons = safeDisabledButtons;
        this.safeDisabledAxes = safeDisabledAxes;
        this.safeScaledAxes = safeScaledAxes;
        this.safeDisabledPOV = safeDisabledPOV;
    }

    // All other methods always fall through to these five

    /**
     * Safe version of {@link GenericHID#getRawButton(int)}.
     *
     * @param button The button to read
     * @return The state of the button
     */
    public boolean getRawButton(int button) {
        if (SafeMode.isEnabled() && safeDisabledButtons.contains(button)) {
            return false;
        } else {
            return unsafeJoystick.getRawButton(button);
        }
    }

    /**
     * Safe version of {@link GenericHID#getRawButtonPressed(int)(int)}.
     *
     * @param button The button to read
     * @return Whether the button was pressed since the last check
     */
    public boolean getRawButtonPressed(int button) {
        if (SafeMode.isEnabled() && safeDisabledButtons.contains(button)) {
            return false;
        } else {
            return unsafeJoystick.getRawButtonPressed(button);
        }
    }

    /**
     * Safe version of {@link GenericHID#getRawButtonReleased(int)(int)}.
     *
     * @param button The button to read
     * @return Whether the button was released since the last check
     */
    public boolean getRawButtonReleased(int button) {
        if (SafeMode.isEnabled() && safeDisabledButtons.contains(button)) {
            return false;
        } else {
            return unsafeJoystick.getRawButtonReleased(button);
        }
    }

    /**
     * Safe version of {@link GenericHID#getRawAxis(int)}.
     *
     * @param axis The axis to read
     * @return The value of the axis
     */
    public double getRawAxis(int axis) {
        if (SafeMode.isEnabled() && safeDisabledAxes.contains(axis)) {
            return 0;
        } else if (SafeMode.isEnabled() && safeScaledAxes.containsKey(axis)) {
            return unsafeJoystick.getRawAxis(axis) * safeScaledAxes.get(axis);
        } else {
            return unsafeJoystick.getRawAxis(axis);
        }
    }

    /**
     * Safe version of {@link GenericHID#getPOV(int)}.
     *
     * @param pov The POV to read
     * @return The value of the POV
     */
    public double getPOV(int pov) {
        if (SafeMode.isEnabled() && safeDisabledPOV.containsKey(pov) && safeDisabledPOV.get(pov).contains(unsafeJoystick.getPOV(pov))) {
            return -1;
        } else {
            return unsafeJoystick.getPOV(pov);
        }
    }

}
