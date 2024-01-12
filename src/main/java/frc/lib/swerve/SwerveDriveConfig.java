package frc.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.controller.FFGains;
import frc.lib.controller.PIDGains;
import frc.robot.Constants;

public class SwerveDriveConfig {

    public CANSparkMax m_driveMotor;
    public SimpleMotorFeedforward m_driveFFController;

    /**
     * @param sparkId   spark Id number - number that was flashed as CAN ID on REV
     *                  hardware client
     * @param PIDValues PID controller values
     * @param FFGains   Feed Forward controller values
     */
    public SwerveDriveConfig(
            int sparkId,
            PIDGains PIDValues,
            FFGains FFValues) {

        m_driveMotor = new CANSparkMax(sparkId, CANSparkMax.MotorType.kBrushless);
        RelativeEncoder enc = m_driveMotor.getEncoder();

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        m_driveMotor.restoreFactoryDefaults();

        // Set the motor direction + or - so that positive is forward
        m_driveMotor.setInverted(Constants.ModuleConstants.kDrivingIsInverted);

        // Set the current limit for the motor to prevent burnout
        m_driveMotor.setSmartCurrentLimit(Constants.ModuleConstants.kDrivingMotorCurrentLimit);

        // Idle mode for the motor; brake or coast
        m_driveMotor.setIdleMode(Constants.ModuleConstants.kDrivingMotorIdleMode);

        /*
         * Apply position and velocity conversion factors for the driving encoder.
         * The native units for position and velocity are rotations and RPM,
         * respectively
         * but we want meters and meters per second to use with WPILib's swerve APIs.
         */
        enc.setPositionConversionFactor(Constants.ModuleConstants.kDrivingEncoderPositionFactor);
        enc.setVelocityConversionFactor(Constants.ModuleConstants.kDrivingEncoderVelocityFactor);

        // Set the PID gains for the driving motor.
        SparkMaxPIDController controller = m_driveMotor.getPIDController();
        controller.setFeedbackDevice(enc);
        controller.setP(PIDValues.P);
        controller.setI(PIDValues.I);
        controller.setD(PIDValues.D);
        controller.setOutputRange(Constants.ModuleConstants.kDrivingMinOutput,
                Constants.ModuleConstants.kDrivingMaxOutput);

        // Set the feedforward gains for the driving motor.
        m_driveFFController = new SimpleMotorFeedforward(
                FFValues.kS,
                FFValues.kV,
                FFValues.kA);

        // Burn the flash to the spark max so that all the settings are saved
        m_driveMotor.burnFlash();
    }

}