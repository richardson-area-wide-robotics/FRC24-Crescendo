package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PivotDirection;

public class Pivot extends SubsystemBase {

    private final CANSparkMax m_PivotLeftMotor;
    private final CANSparkMax m_PivotRightMotor;
    private final AbsoluteEncoder m_PivotEncoder;

    private Measure<Angle> m_setPoint;
    private SparkPIDController m_PivotPIDController;

    private boolean manualControl = false;

    // private ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");
    // private GenericEntry m_setPointEntry = pivotTab.add("Set Point", 0).getEntry();
    // private GenericEntry m_encoderPositionEntry = pivotTab.add("Encoder Position", 0).getEntry();
    // private GenericEntry m_encoderPositionDegreesEntry = pivotTab.add("Encoder Position Degrees", 0).getEntry();
    // private GenericEntry m_atTopLimitEntry = pivotTab.add("At Top Limit", false).getEntry();
    // private GenericEntry m_atBottomLimitEntry = pivotTab.add("At Bottom Limit", false).getEntry();


    /**
     * Config to set basic motor settings to avoid redundancy
     * 
     * @param motor
     */
    public void PivotConfig(CANSparkMax motor, AbsoluteEncoder enc, boolean leader) {
        // Restore the motor to factory settings before any changes was made
        motor.restoreFactoryDefaults();

        // Mode the motor hold when supplied with no commands
        motor.setIdleMode(PivotConstants.pivotIdleMode);

        // Current Limit for motors - prevents brownouts/burnouts
        motor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);

        // Leader motor settings
        if (leader) {
            motor.setInverted(PivotConstants.pivotRightMotorInverted);
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            motor.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.kPivotForwardSoftLimit);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
            motor.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.kPivotReverseSoftLimit);
            // Create and set the PID controller for the motor
            m_PivotPIDController = motor.getPIDController();
            m_PivotPIDController.setFeedbackDevice(enc);
            m_PivotPIDController.setPositionPIDWrappingEnabled(PivotConstants.kPivotPositionPIDWrappingEnabled);
            m_PivotPIDController.setP(PivotConstants.kPivotP);
            m_PivotPIDController.setI(PivotConstants.kPivotI);
            m_PivotPIDController.setD(PivotConstants.kPivotD);
            m_PivotPIDController.setOutputRange(PivotConstants.kPivotMinOutput, PivotConstants.kPivotMaxOutput);
        }

        // Adjust frames to reduce CAN bus traffic, and reduce latency
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);
    }

    public Pivot() {
        m_PivotLeftMotor = new CANSparkMax(PivotConstants.pivotLeftCANID, MotorType.kBrushless);
        m_PivotRightMotor = new CANSparkMax(PivotConstants.pivotRightCANID, MotorType.kBrushless);
        m_PivotEncoder = m_PivotRightMotor.getAbsoluteEncoder();

        PivotConfig(m_PivotRightMotor, m_PivotEncoder, true);
        PivotConfig(m_PivotLeftMotor, m_PivotEncoder, false);

        m_PivotLeftMotor.follow(m_PivotRightMotor, true);

        m_PivotLeftMotor.burnFlash();
        m_PivotRightMotor.burnFlash();

        m_setPoint = Radians.of(getEncoderPosition());

    }

    public boolean bottomLimit() {
        return m_PivotRightMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public boolean topLimit() {
        return m_PivotRightMotor.isSoftLimitEnabled(SoftLimitDirection.kForward);
    }

    public double getDesiredAngle() {
        return m_setPoint.in(Radians);
    }

    public double getEncoderPosition() {
        return m_PivotEncoder.getPosition() ;
    }

    @Override
    public void periodic() {
        // m_setPointEntry.setDouble(m_setPoint);
        // m_encoderPositionEntry.setDouble(getEncoderPosition());
        // m_encoderPositionDegreesEntry.setDouble(getEncoderPosition() * 360);
        // m_atTopLimitEntry.setBoolean(topLimit());
        // m_atBottomLimitEntry.setBoolean(bottomLimit());
        
        // if (m_setPoint > PivotConstants.kPivotMaxAngle) {
        // m_setPoint = PivotConstants.kPivotMaxAngle;
        // } else if (m_setPoint < PivotConstants.kPivotMinAngle) {
        // m_setPoint = PivotConstants.kPivotMinAngle;
        // }

        if (!manualControl)
            pivotTo(m_setPoint);
    }

    public void pivot(PivotDirection direction) {
        switch (direction) {
            case UP:
                manualControl = true;
                pivotSpeed(PivotConstants.kPivotSpeed);
                // m_PivotAngle = m_PivotAngle.plus(Radians.of(0.0005)); // TODO: change to
                // constant
                // if (m_PivotAngle.in(Radians) >=
                // Constants.ShooterConstants.kPivotForwardSoftLimit) {
                // m_PivotAngle = Radians.of(Constants.ShooterConstants.kPivotForwardSoftLimit);
                // }
                // PivotTo(m_PivotAngle);
                break;
            case DOWN:
                manualControl = true;
                pivotSpeed(-PivotConstants.kPivotSpeed);
                // m_PivotAngle = m_PivotAngle.minus(Radians.of(0.0005)); // TODO: change to
                // constant
                // if (m_PivotAngle.in(Radians) <=
                // Constants.ShooterConstants.kPivotReverseSoftLimit) {
                // m_PivotAngle = Radians.of(Constants.ShooterConstants.kPivotReverseSoftLimit);
                // }
                // PivotTo(m_PivotAngle);
                break;
            case STOP:
            default:
                manualControl = false;
                m_setPoint = Radians.of(getEncoderPosition());
                pivotSpeed(0);
        }

    }

    public void pivotSpeed(double speedPercentage) {
        m_PivotRightMotor.set(speedPercentage);
    }

    /**
     * Pivots the shooter to a given angle about the axis of the absolute encoder.
     */
    public void pivotTo(Measure<Angle> angle) {
        m_setPoint = angle;
        m_PivotPIDController.setReference(angle.in(Radians), ControlType.kPosition);
    }

    public void pivotFromCamera(Measure<Angle> angle){
        m_setPoint = angle;
        m_PivotPIDController.setReference((angle.in(Degrees)/360) - 0.0045, ControlType.kPosition);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("atTopLimit", this::topLimit, null);
        builder.addBooleanProperty("atBottomLimit", this::bottomLimit, null);
        builder.addDoubleProperty("desiredPosition", this::getDesiredAngle, null);
        builder.addDoubleProperty("encoderPosition", this::getEncoderPosition, null);
    }

}
