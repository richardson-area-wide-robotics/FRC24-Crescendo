package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder.Type;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ClimberDirection;
import java.util.function.Supplier;

public class Climber extends SubsystemBase {
    private CANSparkMax m_climberLeftMotor;
    private CANSparkMax m_climberRightMotor;

    private SparkPIDController m_climberLeftPIDController;
    private SparkPIDController m_climberRightPIDController;

    private RelativeEncoder m_climberLeftEncoder;
    private RelativeEncoder m_climberRightEncoder;

    public Climber() {
        m_climberLeftMotor = new CANSparkMax(ClimberConstants.kClimberLeftCANID, MotorType.kBrushless);
        m_climberRightMotor = new CANSparkMax(ClimberConstants.kClimberRightCANID, MotorType.kBrushless);
        
        m_climberLeftMotor.restoreFactoryDefaults();
        m_climberLeftMotor.setIdleMode(ClimberConstants.kClimberIdleMode);
        m_climberLeftMotor.setInverted(ClimberConstants.kClimberLeftInverted);
        m_climberLeftMotor.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);
        m_climberLeftMotor.enableSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberForwardSoftLimitEnabled);
        m_climberLeftMotor.enableSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberReverseSoftLimitEnabled);
        m_climberLeftMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberLeftForwardSoftLimit);
        m_climberLeftMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberLeftReverseSoftLimit);
        // m_climberLeftEncoder = m_climberLeftMotor.getEncoder(Type.kHallSensor, 1);
        m_climberLeftPIDController = m_climberLeftMotor.getPIDController();
        m_climberLeftPIDController.setP(ClimberConstants.kLeftP);
        m_climberLeftPIDController.setI(ClimberConstants.kLeftI);
        m_climberLeftPIDController.setD(ClimberConstants.kLeftD);
        m_climberLeftMotor.burnFlash();

        m_climberRightMotor.restoreFactoryDefaults();
        m_climberRightMotor.setIdleMode(ClimberConstants.kClimberIdleMode);
        m_climberRightMotor.setInverted(ClimberConstants.kClimberRightInverted);
        m_climberRightMotor.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);
        m_climberRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberForwardSoftLimitEnabled);
        m_climberRightMotor.enableSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberReverseSoftLimitEnabled);
        m_climberRightMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberRightForwardSoftLimit);
        m_climberRightMotor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberRightReverseSoftLimit);
        m_climberRightPIDController = m_climberRightMotor.getPIDController();
        m_climberRightPIDController.setP(ClimberConstants.kRightP);
        m_climberRightPIDController.setI(ClimberConstants.kRightI);
        m_climberRightPIDController.setD(ClimberConstants.kRightD);
        m_climberRightMotor.burnFlash();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void idle() {
        m_climberLeftMotor.set(0);
        m_climberRightMotor.set(0);
    }

    /**
     * Level the robot (to roll = 0 degrees) using a climber
     * @param rollSupplier
     */
    public void level(Supplier<Measure<Angle>> rollSupplier) {
        // only manipulate one side of the climber at a time
        boolean useLeft = rollSupplier.get().in(Degrees) > 0;
        while (Math.abs(rollSupplier.get().in(Degrees)) > ClimberConstants.kRollTolerance.in(Degrees)) {
            if (rollSupplier.get().in(Degrees) > 0) {
                if (useLeft) {
                    m_climberLeftMotor.set(ClimberConstants.kLevelSpeed);
                } else {
                    m_climberRightMotor.set(-ClimberConstants.kLevelSpeed);
                }
            } else if (rollSupplier.get().in(Degrees) < 0) {
                if (useLeft) {
                    m_climberLeftMotor.set(-ClimberConstants.kLevelSpeed);
                } else {
                    m_climberRightMotor.set(ClimberConstants.kLevelSpeed);
                }
            }
        }
    }

    public void setLeftDirection(ClimberDirection direction) {
        switch (direction) {
            case UP:
                setLeftSpeed(ClimberConstants.kClimbSpeed);
                break;
            case DOWN:
                setLeftSpeed(-ClimberConstants.kClimbSpeed);
                break;
        }
    }

    public void setLeftSpeed(double speed) {
        m_climberLeftMotor.set(speed);
    }

    public void setRightDirection(ClimberDirection direction) {
        switch (direction) {
            case UP:
                setRightSpeed(ClimberConstants.kClimbSpeed);
                break;
            case DOWN:
                setRightSpeed(-ClimberConstants.kClimbSpeed);
                break;
        }
    }

    public void setRightSpeed(double speed) {
        m_climberRightMotor.set(speed);
    }

    public void setDirection(ClimberDirection direction) {
        switch (direction) {
            case UP:
                setSpeed(ClimberConstants.kClimbSpeed);
                break;
            case DOWN:
                setSpeed(-ClimberConstants.kClimbSpeed);
                break;
        }
    }

    public void setSpeed(double speed) {
        m_climberLeftMotor.set(speed);
        m_climberRightMotor.set(speed);
    }

    public void stop() {
        m_climberLeftMotor.stopMotor();
        m_climberRightMotor.stopMotor();
    }
}
