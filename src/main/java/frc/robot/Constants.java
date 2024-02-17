// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Distance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;
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
    public static final int feederMotorCANID = 11;
    public static final int kickerMotorCANID = 12;
    public static final int shooterRightCANID = 13;
    public static final int shooterLeftCANID = 14;

    public static final int pivotRightCANID = 9;
    public static final int pivotLeftCANID = 10;

    public static final int feederMotorCurrentLimit = 40;
    public static final int kickerMotorCurrentLimit = 40;
    public static final int shooterRightMotorCurrentLimit = 40;
    public static final int shooterLeftMotorCurrentLimit = 40;
    public static final int pivotRightMotorCurrentLimit = 40;
    public static final int pivotLeftMotorCurrentLimit = 40;

    public static final boolean pivotRightMotorInverted = true;

    public static final Measure<Distance> horizontalNoteCompression = Inches.of(12.0);
    public static final Measure<Distance> shooterWheelRadius = Inches.of(2.0);
    public static final Measure<Velocity<Angle>> kickerSpeed = RPM.of(300.0);

    public static final Measure<Velocity<Distance>> launchSpeedTolerance = MetersPerSecond.of(0.01); 

    public static class Pivot {
      public static final double P = 0.0007;
      public static final double I = 0.0;
      public static final double D = 0.001;

      public static final Measure<Angle> toleranceAngle = Degrees.of(3.6); 

      // smallest angle (between hardstop and shooter) that the shooter can pivot to
      public static final Measure<Angle> minAngle = Degrees.of(5.0); // TBD

      // highest angle (between hardstop and shooter) that the shooter can pivot to
      public static final Measure<Angle> maxAngle = Degrees.of(45.0); // TBD
    }

    public static class Feeder {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
    }

    public static class Kicker {
      public static final double P = 0;
      public static final double I = 0;
      public static final double D = 0;
    }
    
    public static class Shooter {
      public static final double P = 0.0007;
      public static final double I = 0.0;
      public static final double D = 0.001;
    }

  }
}