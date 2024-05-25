package org.carlmontrobotics.lib199.safeMode;

import static org.junit.Assert.*;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import org.carlmontrobotics.lib199.testUtils.TestRules;
import org.junit.ClassRule;
import org.junit.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SafeModeTest {

    @ClassRule
    public static TestRules.InitializeHAL classRule = new TestRules.InitializeHAL(); 
    // Test SafeMode.java using JUnit

    @Test
    public void testSafeModeEnableDisable() {
        SafeMode.enable();
        assertTrue(SafeMode.isEnabled());
        assertTrue(SmartDashboard.getBoolean("Safe Mode", false));

        SafeMode.disable();
        assertFalse(SafeMode.isEnabled());
        assertFalse(SmartDashboard.getBoolean("Safe Mode", true));

        SmartDashboard.putBoolean("Safe Mode", false);
        assertFalse(SafeMode.isEnabled());
        assertFalse(SmartDashboard.getBoolean("Safe Mode", true));

        SmartDashboard.putBoolean("Safe Mode", true);
        assertTrue(SafeMode.isEnabled());
        assertTrue(SmartDashboard.getBoolean("Safe Mode", false));
    }

    @Test
    public void testCallbacks() {
        AtomicInteger enabledCounter = new AtomicInteger(0);
        AtomicInteger disabledCounter = new AtomicInteger(0);

        SafeMode.disable(); // The callbacks only get called when the state changes, so we need to start in a known state

        SafeMode.onEnabled(() -> enabledCounter.incrementAndGet());
        SafeMode.onDisabled(() -> disabledCounter.incrementAndGet());

        SafeMode.enable();
        CommandScheduler.getInstance().run();
        assertEquals(1, enabledCounter.get());
        assertEquals(0, disabledCounter.get());

        SafeMode.disable();
        CommandScheduler.getInstance().run();
        assertEquals(1, enabledCounter.get());
        assertEquals(1, disabledCounter.get());
    }

    @Test
    public void testSafeConstants() {
        Supplier<String> safeString = SafeMode.constant("normal", "safe");
        BooleanSupplier safeBoolean = SafeMode.constant(false, true);
        DoubleSupplier safeDouble = SafeMode.constant(1.0, 2.0);
        IntSupplier safeInt = SafeMode.constant(1, 2);
        LongSupplier safeLong = SafeMode.constant(1L, 2L);

        SafeMode.disable();
        assertEquals("normal", safeString.get());
        assertEquals(false, safeBoolean.getAsBoolean());
        assertEquals(1.0, safeDouble.getAsDouble(), 0.0);
        assertEquals(1, safeInt.getAsInt());
        assertEquals(1L, safeLong.getAsLong());

        SafeMode.enable();
        assertEquals("safe", safeString.get());
        assertEquals(true, safeBoolean.getAsBoolean());
        assertEquals(2.0, safeDouble.getAsDouble(), 0.0);
        assertEquals(2, safeInt.getAsInt());
        assertEquals(2L, safeLong.getAsLong());
    }


}
