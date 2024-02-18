package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.swerve.SwerveModule;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;

public class MAXSwerveModule implements SwerveModule {
  private CANSparkFlex m_drivingMotor;
  private CANSparkMax m_turningMotor;

  private RelativeEncoder m_drivingEncoder;
  private AbsoluteEncoder m_turningEncoder;

  private SparkPIDController m_drivingPIDController;
  private SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public void drivingConfig(
      CANSparkFlex driveMotor, SparkPIDController drivingPidController, RelativeEncoder enc) {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    driveMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    enc = driveMotor.getEncoder();

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    enc.setPositionConversionFactor(Constants.ModuleConstants.kDrivingEncoderPositionFactor);
    enc.setVelocityConversionFactor(Constants.ModuleConstants.kDrivingEncoderVelocityFactor);

    // Set the PID gains for the driving motor.
    drivingPidController = driveMotor.getPIDController();
    drivingPidController.setFeedbackDevice(enc);
    drivingPidController.setP(Constants.ModuleConstants.kDrivingPIDGains.P);
    drivingPidController.setI(Constants.ModuleConstants.kDrivingPIDGains.I);
    drivingPidController.setD(Constants.ModuleConstants.kDrivingPIDGains.D);
    drivingPidController.setFF(Constants.ModuleConstants.kDrivingFF);
    drivingPidController.setOutputRange(
        Constants.ModuleConstants.kDrivingMinOutput, Constants.ModuleConstants.kDrivingMaxOutput);

      driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
      driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
      driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);

    driveMotor.setIdleMode(Constants.ModuleConstants.kDrivingMotorIdleMode);
    driveMotor.setSmartCurrentLimit(Constants.ModuleConstants.kDrivingMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    driveMotor.burnFlash();
  }

  public void turningConfig(
      CANSparkMax turnMotor, SparkPIDController turningPidController, AbsoluteEncoder enc) {
    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    turnMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    enc = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    enc.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderPositionFactor);
    enc.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    enc.setInverted(Constants.ModuleConstants.kTurningEncoderInverted);

    // Set the PID gains for the driving motor.
    turningPidController = turnMotor.getPIDController();
    turningPidController.setFeedbackDevice(enc);
    turningPidController.setP(Constants.ModuleConstants.kTurningPIDGains.P);
    turningPidController.setI(Constants.ModuleConstants.kTurningPIDGains.I);
    turningPidController.setD(Constants.ModuleConstants.kTurningPIDGains.D);
    turningPidController.setFF(Constants.ModuleConstants.kTurningFF);
    turningPidController.setOutputRange(
        Constants.ModuleConstants.kTurningMinOutput, Constants.ModuleConstants.kTurningMaxOutput);
    
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPidController.setPositionPIDWrappingEnabled(true);
    turningPidController.setPositionPIDWrappingMinInput(
        Constants.ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningPidController.setPositionPIDWrappingMaxInput(
        Constants.ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    turnMotor.setIdleMode(Constants.ModuleConstants.kTurningMotorIdleMode);

    turnMotor.setSmartCurrentLimit(Constants.ModuleConstants.kTurningMotorCurrentLimit);

    turnMotor.burnFlash();
  }

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(SwerveModuleConstants constants) {
    // create turning and driving motors
    m_drivingMotor = new CANSparkFlex(constants.driveMotorID, MotorType.kBrushless);
    m_drivingEncoder = m_drivingMotor.getEncoder();
    m_drivingPIDController = m_drivingMotor.getPIDController();
    drivingConfig(m_drivingMotor, m_drivingPIDController, m_drivingEncoder);

    m_turningMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPIDController = m_turningMotor.getPIDController();
    turningConfig(m_turningMotor, m_turningPIDController, m_turningEncoder);

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