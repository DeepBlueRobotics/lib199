package org.carlmontrobotics.lib199;

import org.carlmontrobotics.lib199.sim.MockedCANCoder;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A class containing methods to create and configure sensors.
 */
public class SensorFactory {

    /**
     * Creates a CANCoder object, linking it to the simulator if necessary
     *
     * @param port The CAN ID of the CANCoder
     * @return The CANCoder object
     */
    public static CANCoder createCANCoder(int port) {
        CANCoder canCoder = new CANCoder(port);
        if (RobotBase.isSimulation())
            new MockedCANCoder(canCoder);
        return canCoder;
    }

    /**
     * Configures a USB Camera.
     * See {@link CameraServer#startAutomaticCapture} for more details.
     * This MUST be called AFTER AHRS initialization or the code will be unable to
     * connect to the gyro.
     *
     * @return The configured camera
     */
    public static UsbCamera configureCamera() {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        CameraServer.getServer().setSource(camera);
        return camera;
    }

    /**
     * This method is equivalent to calling {@link #configureCamera()}
     * {@code numCameras} times.
     * The last camera will be set as the primary Camera feed.
     * To change it, call {@code CameraServer.getServer().setSource()}.
     *
     * @param numCameras The number of cameras to configure
     * @return The configured cameras.
     */
    public static UsbCamera[] configureCameras(int numCameras) {
        UsbCamera[] cameras = new UsbCamera[numCameras];
        for (int i = 0; i < numCameras; i++)
            cameras[i] = configureCamera();
        return cameras;
    }

}
