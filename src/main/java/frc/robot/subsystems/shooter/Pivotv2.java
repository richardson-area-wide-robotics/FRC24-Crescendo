package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.Pivot;

public class Pivotv2 extends SubsystemBase{

    private final CANSparkMax m_pivotLeftMotor;
    private final CANSparkMax m_pivotRightMotor;
    private final AbsoluteEncoder m_pivotEncoder;

    private double targetAngle;

    private SparkPIDController m_pivotPIDController;

    /**
     * Config to set basic motor settings to avoid redundancy
     * @param motor
     */
    public void PivotConfig(CANSparkMax motor, AbsoluteEncoder enc, boolean leader){
        // Restore the motor to factory settings before any changes was made
        motor.restoreFactoryDefaults();

        // Mode the motor hold when supplied with no commands 
        motor.setIdleMode(Constants.ShooterConstants.Pivot.pivotIdleMode);

        // Current Limit for motors - prevents brownouts/burnouts
        motor.setSmartCurrentLimit(Constants.ShooterConstants.Pivot.pivotCurrentLimit);
        
        // Leader motor settings
        if (leader){
        motor.setInverted(Constants.ShooterConstants.Pivot.pivotRightMotorInverted);
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, Constants.ShooterConstants.Pivot.kPivotForwardSoftLimit);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true );
        motor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ShooterConstants.Pivot.kPivotReverseSoftLimit);
        // Create and set the PID controller for the motor  
        m_pivotPIDController = motor.getPIDController();
        m_pivotPIDController.setFeedbackDevice(enc);
        m_pivotPIDController.setPositionPIDWrappingEnabled(Constants.ShooterConstants.Pivot.kPivotPositionPIDWrappingEnabled);
        m_pivotPIDController.setP(Constants.ShooterConstants.Pivot.kPivotP);
        m_pivotPIDController.setI(Constants.ShooterConstants.Pivot.kPivotI);
        m_pivotPIDController.setD(Constants.ShooterConstants.Pivot.kPivotD);
        m_pivotPIDController.setOutputRange(Constants.ShooterConstants.Pivot.kPivotMinOutput, Constants.ShooterConstants.Pivot.kPivotMaxOutput);
        }

        // Adjust frames to reduce CAN bus traffic, and reduce latency
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);
    }

    public Pivotv2() {
        m_pivotLeftMotor = new CANSparkMax(Constants.ShooterConstants.Pivot.pivotLeftCANID, MotorType.kBrushless);
        m_pivotRightMotor = new CANSparkMax(Constants.ShooterConstants.Pivot.pivotRightCANID, MotorType.kBrushless);
        m_pivotEncoder = m_pivotRightMotor.getAbsoluteEncoder();
        
        PivotConfig(m_pivotRightMotor, m_pivotEncoder, true);
        PivotConfig(m_pivotLeftMotor, m_pivotEncoder, false);

        m_pivotLeftMotor.follow(m_pivotRightMotor, true);

        m_pivotLeftMotor.burnFlash();
        m_pivotRightMotor.burnFlash();

        targetAngle = getEncoderPosition();
       
    }

    public boolean bottomLimit(){
        return m_pivotRightMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public boolean topLimit(){
        return m_pivotRightMotor.isSoftLimitEnabled(SoftLimitDirection.kForward);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atTopLimit", this::bottomLimit, null);
        builder.addDoubleProperty("desiredPosition", this::getDesiredAngle, null);
        builder.addDoubleProperty("encoderPosition", this::getEncoderPosition, null);
    }

    public void pivotTo(double angle){
        m_pivotPIDController.setReference(angle, ControlType.kPosition);
    }

    public double getDesiredAngle(){
        return targetAngle;
    }

    public double getEncoderPosition(){
        return m_pivotEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // if (targetAngle > Pivot.kPivotForwardSoftLimit) {
        //     targetAngle = Pivot.kPivotForwardSoftLimit;
        //   } else if (targetAngle < Pivot.kPivotReverseSoftLimit) {
        //     targetAngle = Pivot.kPivotReverseSoftLimit;
        //   }
        // This method will be called once per scheduler run
        pivotTo(targetAngle);
    }
    
}
