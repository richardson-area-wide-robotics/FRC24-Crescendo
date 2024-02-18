package frc.lib.util;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final double angleOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param angleOffset
   */
  public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.angleOffset = angleOffset;
  }
}