package org.carlmontrobotics.lib199.sim;

import java.util.HashMap;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;

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
    private SimDouble motorId;
    private SimDouble motorType;
    private SimDouble gearing;
    private CANCoderSimCollection sim;

    public MockedCANCoder(CANCoder canCoder) {
        port = canCoder.getDeviceID();
        device = SimDevice.create("CANCoder", port);
        position = device.createDouble("count", Direction.kInput, 0);
        motorId = device.createDouble("motorId", Direction.kOutput, -1);
        motorType = device.createDouble("motorType", Direction.kOutput, MotorType.NONE.id);
        gearing = device.createDouble("gearing", Direction.kOutput, 1);
        sim = canCoder.getSimCollection();
        Lib199Subsystem.registerPeriodic(this::update);
        sims.put(port, this);
    }

    public void update() {
        sim.setRawPosition((int) (position.get() * kCANCoderCPR));
    }

    public void linkMotor(int motorId, MotorType motorType, double gearing) {
        this.motorId.set(motorId);
        this.motorType.set(motorType.id);
        this.gearing.set(gearing);
    }

    public static void linkMotor(int canCoderPort, int motorId, MotorType motorType, double gearing) {
        if(sims.containsKey(canCoderPort)) sims.get(canCoderPort).linkMotor(motorId, motorType, gearing);
    }

    public static enum MotorType {
        NONE(0), TALON(1), VICTOR(2), SPARKMAX(3);

        public final int id;

        private MotorType(int id) {
            this.id = id;
        }
    }

}