package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.PivotDirection;

public class Pivot extends SubsystemBase {
    // TODO: move to separate subsystem folder
    private CANSparkMax m_pivotLeftMotor;
    private CANSparkMax m_pivotRightMotor;

    private SparkPIDController m_pivotPIDController;
    private AbsoluteEncoder m_pivotEncoder;

    private Measure<Angle> m_pivotAngle;

    public Pivot() {
        m_pivotLeftMotor = new CANSparkMax(Constants.ShooterConstants.pivotLeftCANID, MotorType.kBrushless);
        m_pivotRightMotor = new CANSparkMax(Constants.ShooterConstants.pivotRightCANID, MotorType.kBrushless);

        m_pivotLeftMotor.restoreFactoryDefaults();
        m_pivotLeftMotor.setSmartCurrentLimit(Constants.ShooterConstants.pivotLeftMotorCurrentLimit);
        m_pivotLeftMotor.follow(m_pivotRightMotor, true);
       
        m_pivotRightMotor.restoreFactoryDefaults();
        m_pivotEncoder = m_pivotRightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_pivotPIDController = m_pivotRightMotor.getPIDController();
        m_pivotPIDController.setFeedbackDevice(m_pivotEncoder);
        m_pivotPIDController.setPositionPIDWrappingEnabled(false);
        m_pivotPIDController.setP(Constants.ShooterConstants.kPivotP);
        m_pivotPIDController.setI(Constants.ShooterConstants.kPivotI);
        m_pivotPIDController.setD(Constants.ShooterConstants.kPivotD);
        m_pivotRightMotor.setIdleMode(IdleMode.kBrake);
        m_pivotRightMotor.setInverted(Constants.ShooterConstants.kPivotRightMotorInverted);
        m_pivotRightMotor.setSmartCurrentLimit(Constants.ShooterConstants.pivotRightMotorCurrentLimit);
        m_pivotRightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_pivotRightMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ShooterConstants.kPivotForwardSoftLimit);
        m_pivotRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_pivotRightMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ShooterConstants.kPivotReverseSoftLimit);
        m_pivotRightMotor.burnFlash();
        m_pivotLeftMotor.burnFlash();

        m_pivotAngle = Radians.of(m_pivotEncoder.getPosition());
    }

    @Override
    public void periodic() {
        pivotTo(m_pivotAngle);
    }

    public void pivot(PivotDirection direction) {
        switch (direction) {
            case UP:
                // pivotSpeed(Constants.ShooterConstants.kPivotSpeed);
                m_pivotAngle = m_pivotAngle.plus(Radians.of(0.0005)); // TODO: change to constant
                if (m_pivotAngle.in(Radians) >= Constants.ShooterConstants.kPivotForwardSoftLimit) {
                    m_pivotAngle = Radians.of(Constants.ShooterConstants.kPivotForwardSoftLimit);
                }
                pivotTo(m_pivotAngle);
                break;
            case DOWN:
                // pivotSpeed(-Constants.ShooterConstants.kPivotSpeed);
                m_pivotAngle = m_pivotAngle.minus(Radians.of(0.0005)); // TODO: change to constant
                if (m_pivotAngle.in(Radians) <= Constants.ShooterConstants.kPivotReverseSoftLimit) {
                    m_pivotAngle = Radians.of(Constants.ShooterConstants.kPivotReverseSoftLimit);
                }
                pivotTo(m_pivotAngle);
                break;
            case STOP:
            default:
                pivotSpeed(0);
        }
    }

    public void pivotSpeed(double speedPercentage) {
        m_pivotRightMotor.set(speedPercentage);
    }

    /** 
     * Pivots the shooter to a given angle about the axis of the absolute encoder. 
     */
    public void pivotTo(Measure<Angle> angle) {
        m_pivotAngle = angle;
        m_pivotPIDController.setReference(angle.in(Radians), ControlType.kPosition);
    }

    public void pivotTo(double radians) {
        pivotTo(Radians.of(radians));
    }

}
