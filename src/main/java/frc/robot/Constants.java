// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Distance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Measure;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // TODO: shooter constants
  public static class ShooterConstants {
    public static final int feederMotorCANID = 9;
    public static final int kickerMotorCANID = 10;
    public static final int shooterRightCANID = 11;
    public static final int shooterLeftCANID = 12;

    // TODO: fix import error
    public static final Measure<Distance> shooterWheelDiameter = units.Units.Inches.of(4.0);
  }
}
