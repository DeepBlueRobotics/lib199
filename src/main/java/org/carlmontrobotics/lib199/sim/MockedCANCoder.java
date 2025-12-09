package org.carlmontrobotics.lib199.sim;

import java.util.HashMap;

import org.carlmontrobotics.lib199.ErrorCodeAnswer;
import org.carlmontrobotics.lib199.Mocks;
import org.carlmontrobotics.lib199.REVLibErrorAnswer;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.simulation.SimValueCallback;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class MockedCANCoder {

    private static final HashMap<Integer, MockedCANCoder> sims = new HashMap<>();

    private int port;
    private SimDevice device;
    private SimDeviceSim deviceSim;
    private SimDouble position; // Rotations - Continuous
    private CANcoderSimState sim;
    private CANcoderConfigurator configurator;

    public MockedCANCoder(CANcoder canCoder) {
        port = canCoder.getDeviceID();
        device = SimDevice.create("CANDutyCycle:CANCoder", port);
        position = device.createDouble("position", Direction.kInput, 0);
        sim = canCoder.getSimState();
        configurator = canCoder.getConfigurator();
        deviceSim = new SimDeviceSim("CANDutyCycle:CANCoder", port);
        deviceSim.registerValueChangedCallback(position, new SimValueCallback() {
            @Override
            public void callback(String name, int handle, int direction, HALValue value) {
                sim.setRawPosition(value.getDouble());
            }
        }, true);
        sims.put(port, this);
    }

    public CANcoderConfigurator getConfigurator() {
        return configurator;
    }

    public static CANcoder createMock(CANcoder canCoder) {
        return Mocks.createMock(CANcoder.class, new MockedCANCoder(canCoder));
    }

}