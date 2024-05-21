package org.carlmontrobotics.lib199.sim;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

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
    private SimDouble gearing;
    private SimBoolean init;
    private CANcoderSimState sim;

    public MockedCANCoder(CANcoder canCoder) {
        port = canCoder.getDeviceID();
        device = SimDevice.create("CANDutyCycle:CANCoder", port);
        position = device.createDouble("position", Direction.kInput, 0);
        gearing = device.createDouble("gearing", Direction.kOutput, 1);
        sim = canCoder.getSimState();
        deviceSim = new SimDeviceSim("CANDutyCycle:CANCoder", port);
        deviceSim.registerValueChangedCallback(position, new SimValueCallback() {
            @Override
            public void callback(String name, int handle, int direction, HALValue value) {
                sim.setRawPosition(value.getDouble());
            }
        }, true);
        sims.put(port, this);
        init = device.createBoolean("init", Direction.kOutput, true);
    }

    public void setGearing(double gearing) {
        this.gearing.set(gearing);
    }

    public static void setGearing(int port, double gearing) {
        if(sims.containsKey(port)) sims.get(port).setGearing(gearing);
    }

}