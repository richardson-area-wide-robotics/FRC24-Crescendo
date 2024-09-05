package frc.lib.swerve;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final double angleOffset;

  /**
   * Store configuration for a swerve module, 
   *
   * @param driveMotorID  The ID of the drive motor (Forward-backword)
   * @param angleMotorID  The ID of the angle motor (Left-right)
   * @param angleOffset   The angle offset of the module
   */
  public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.angleOffset = angleOffset;
  }
}