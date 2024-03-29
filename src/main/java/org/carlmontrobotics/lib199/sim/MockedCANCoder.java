package org.carlmontrobotics.lib199.sim;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import org.carlmontrobotics.lib199.Lib199Subsystem;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class MockedCANCoder {

    public static final double kCANCoderCPR = 4096;

    private static final HashMap<Integer, MockedCANCoder> sims = new HashMap<>();

    private int port;
    private SimDevice device;
    private SimDouble position; // Rotations - Continuous
    private SimDouble gearing;
    private CANcoderSimState sim;

    public MockedCANCoder(CANcoder canCoder) {
        port = canCoder.getDeviceID();
        device = SimDevice.create("CANCoder", port);
        position = device.createDouble("count", Direction.kInput, 0);
        gearing = device.createDouble("gearing", Direction.kOutput, 1);
        sim = canCoder.getSimState();
        Lib199Subsystem.registerAsyncSimulationPeriodic(this::update);
        sims.put(port, this);
    }

    public void update() {
        sim.setRawPosition(position.get() / kCANCoderCPR);
    }

    public void setGearing(double gearing) {
        this.gearing.set(gearing);
    }

    public static void setGearing(int port, double gearing) {
        if(sims.containsKey(port)) sims.get(port).setGearing(gearing);
    }

}