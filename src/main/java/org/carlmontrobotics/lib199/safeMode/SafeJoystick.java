package org.carlmontrobotics.lib199.safeMode;

import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID;

public class SafeJoystick {

    public final GenericHID unsafeJoystick;

    private final Set<Integer> safeDisabledButtons;
    private final Set<Integer> safeDisabledAxes;
    private final Map<Integer, Double> safeScaledAxes;
    private final Map<Integer, Set<Integer>> safeDisabledPOV;

    // Use AtomicBoolean so that the boolean is passed by reference
    public SafeJoystick(GenericHID unsafeJoystick, Set<Integer> safeDisabledButtons, Set<Integer> safeDisabledAxes, Map<Integer, Double> safeScaledAxes, Map<Integer, Set<Integer>> safeDisabledPOV) {
        this.unsafeJoystick = unsafeJoystick;
        this.safeDisabledButtons = safeDisabledButtons;
        this.safeDisabledAxes = safeDisabledAxes;
        this.safeScaledAxes = safeScaledAxes;
        this.safeDisabledPOV = safeDisabledPOV;
    }

    // All other methods always fall through to these five

    public boolean getRawButton(int button) {
        if (SafeMode.isEnabled() && safeDisabledButtons.contains(button)) {
            return false;
        } else {
            return unsafeJoystick.getRawButton(button);
        }
    }

    public boolean getRawButtonPressed(int button) {
        if (SafeMode.isEnabled() && safeDisabledButtons.contains(button)) {
            return false;
        } else {
            return unsafeJoystick.getRawButtonPressed(button);
        }
    }

    public boolean getRawButtonReleased(int button) {
        if (SafeMode.isEnabled() && safeDisabledButtons.contains(button)) {
            return false;
        } else {
            return unsafeJoystick.getRawButtonReleased(button);
        }
    }

    public double getRawAxis(int axis) {
        if (SafeMode.isEnabled() && safeDisabledAxes.contains(axis)) {
            return 0;
        } else if (SafeMode.isEnabled() && safeScaledAxes.containsKey(axis)) {
            return unsafeJoystick.getRawAxis(axis) * safeScaledAxes.get(axis);
        } else {
            return unsafeJoystick.getRawAxis(axis);
        }
    }

    public double getPOV(int pov) {
        if (SafeMode.isEnabled() && safeDisabledPOV.containsKey(pov) && safeDisabledPOV.get(pov).contains(unsafeJoystick.getPOV(pov))) {
            return -1;
        } else {
            return unsafeJoystick.getPOV(pov);
        }
    }

}