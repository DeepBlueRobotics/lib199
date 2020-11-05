/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // Move the jnilib files manually to tmp/jniExtractDir since it didn't happen automatically
    if (System.getProperty("os.name").equals("Mac OS X")) {
      String dir = System.getProperty("user.dir");
      String files[] = {"libJavaController.jnilib", "libvehicle.jnilib"};
      Path src, dest;
      for (String s : files) {
        System.load(dir + "/libs/jnilibs/" + s);
        try {
          src = Path.of(dir + "/libs/jnilibs/" + s);
          dest = Path.of(dir + "/build/tmp/jniExtractDir/" + s);
          if (!Files.exists(dest)) Files.createFile(dest);
          Files.copy(src, dest, StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException io) {
          io.printStackTrace();
          System.exit(1);
        }
      }
    }
    
    RobotBase.startRobot(Robot::new);
  }
}
