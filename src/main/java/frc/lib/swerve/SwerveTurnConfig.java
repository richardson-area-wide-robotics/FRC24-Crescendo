package frc.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.controller.FFGains;
import frc.lib.controller.PIDGains;
import frc.robot.Constants;

public class SwerveTurnConfig {

    public CANSparkMax turnMotor;
    public ArmFeedforward FFController;

    public SwerveTurnConfig(
            int turnMotorID,
            PIDGains PIDValues,
            FFGains FFValues,
            int currentLimit) {

        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        SparkAbsoluteEncoder enc = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        turnMotor.restoreFactoryDefaults();

        // Set the current limit for the motor to prevent burnout
        turnMotor.setSmartCurrentLimit(Constants.ModuleConstants.kTurningMotorCurrentLimit);

        // Idle mode for the motor; brake or coast
        turnMotor.setIdleMode(Constants.ModuleConstants.kTurningMotorIdleMode);

        // Set the motor direction + or - so that positive is forward
        enc.setInverted(Constants.ModuleConstants.kTurningEncoderInverted);

        /*
         * Set the periodic frame periods to ensure that the CAN bus is not saturated.
         */
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
        turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);

        /*
         * Apply position and velocity conversion factors for the turning encoder.
         * The native units for position and velocity are rotations and RPM,
         * respectively
         * but we want radians and radians per second to use with WPILib's swerve APIs.
         */
        enc.setPositionConversionFactor(Constants.ModuleConstants.kTurningEncoderPositionFactor);
        enc.setVelocityConversionFactor(Constants.ModuleConstants.kTurningEncoderVelocityFactor);

        // Set the PID gains for the driving motor.
        SparkPIDController controller = turnMotor.getPIDController();
        controller.setFeedbackDevice(enc);
        controller.setOutputRange(Constants.ModuleConstants.kTurningMinOutput,
                Constants.ModuleConstants.kTurningMaxOutput);
        controller.setPositionPIDWrappingEnabled(true);
        controller.setPositionPIDWrappingMinInput(Constants.ModuleConstants.kTurningEncoderPositionPIDMinInput);
        controller.setPositionPIDWrappingMaxInput(Constants.ModuleConstants.kTurningEncoderPositionPIDMaxInput);
        controller.setP(PIDValues.P);
        controller.setI(PIDValues.I);
        controller.setD(PIDValues.D);

        // Set the feedforward gains for the driving motor.
        FFController = new ArmFeedforward(
                FFValues.kS,
                0.0,
                FFValues.kV,
                FFValues.kA);

        // Burn the flash to make sure all settings are saved
        turnMotor.burnFlash();
    }
}