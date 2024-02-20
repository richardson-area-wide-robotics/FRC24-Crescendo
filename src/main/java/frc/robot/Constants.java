// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ClimberConstants {
    public static final int kClimberLeftCANID = 0;
    public static final int kClimberRightCANID = 1;

    public static final IdleMode kClimberIdleMode = IdleMode.kBrake;

    public static final boolean kClimberLeftInverted = false;
    public static final boolean kClimberRightInverted = true;

    public static final int kClimberCurrentLimit = 40;

    public static final boolean kClimberForwardSoftLimitEnabled = true;
    public static final boolean kClimberReverseSoftLimitEnabled = true;

    public static final float kClimberLeftForwardSoftLimit = 0;
    public static final float kClimberRightForwardSoftLimit = 0;

    public static final float kClimberLeftReverseSoftLimit = 0;
    public static final float kClimberRightReverseSoftLimit = 0;

    public static final double kLeftP = 0.1;
    public static final double kLeftI = 0.0;
    public static final double kLeftD = 0.0;

    public static final double kRightP = 0.1;
    public static final double kRightI = 0.0;
    public static final double kRightD = 0.0;
    
    public static final double kClimbSpeed = 0.5;
    public static final double kLevelSpeed = 0.2;
    public static final Measure<Angle> kRollTolerance = Degrees.of(5.0);

    public enum ClimberDirection {
      UP,
      DOWN
    }
  }
}
