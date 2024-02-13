package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.swerve.SwerveDriveConfig;
import frc.lib.swerve.SwerveModule;
import frc.lib.swerve.SwerveTurnConfig;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class MAXSwerveModule implements SwerveModule {
  private CANSparkMax m_drivingMotor;
  private CANSparkMax m_turningMotor;

  private RelativeEncoder m_drivingEncoder;
  private AbsoluteEncoder m_turningEncoder;

  private SwerveDriveConfig driveConfig;
  private SwerveTurnConfig turningConfig;

  private SparkMaxPIDController m_drivingPIDController;
  private SparkMaxPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(SwerveModuleConstants constants) {
    // create turning and driving motors
    m_drivingMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_drivingPIDController = m_drivingMotor.getPIDController();
    driveConfig = new SwerveDriveConfig(constants.driveMotorID, Constants.ModuleConstants.kDrivingPIDGains, Constants.ModuleConstants.kDrivingFFGains);

    m_turningMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPIDController = m_turningMotor.getPIDController();
    turningConfig = new SwerveTurnConfig(constants.angleMotorID, Constants.ModuleConstants.kTurningPIDGains, Constants.ModuleConstants.kTurningFFGains, Constants.ModuleConstants.kTurningMotorCurrentLimit);

    m_chassisAngularOffset = constants.angleOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  @Override
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  @Override
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  @Override
  public double getDriveDistanceMeters() {
    return m_drivingEncoder.getPosition();
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  @Override
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {}
}