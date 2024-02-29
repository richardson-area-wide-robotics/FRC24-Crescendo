package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Angle;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public void climberConfig(CANSparkMax motor, boolean climberLeftSide){
        motor.restoreFactoryDefaults();

        motor.setIdleMode(ClimberConstants.kClimberIdleMode);

        if(climberLeftSide) {
            motor.setInverted(ClimberConstants.kClimberLeftInverted);
            m_climberLeftEncoder = motor.getEncoder();
            m_climberLeftPIDController = motor.getPIDController();
            m_climberLeftPIDController.setP(ClimberConstants.kClimberP);
            m_climberLeftPIDController.setI(ClimberConstants.kClimberI);
            m_climberLeftPIDController.setD(ClimberConstants.kClimberD);
        }

        if (!climberLeftSide) { 
            motor.setInverted(ClimberConstants.kClimberRightInverted);
            m_climberRightEncoder = motor.getEncoder();
            m_climberRightPIDController = motor.getPIDController();
            m_climberRightPIDController.setP(ClimberConstants.kClimberP);
            m_climberRightPIDController.setI(ClimberConstants.kClimberI);
            m_climberRightPIDController.setD(ClimberConstants.kClimberD);
        }

        motor.setSmartCurrentLimit(ClimberConstants.kClimberCurrentLimit);

        motor.enableSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberForwardSoftLimitEnabled);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberReverseSoftLimitEnabled);
        motor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kClimberForwardSoftLimit);
        motor.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.kClimberReverseSoftLimit);

        motor.burnFlash();
    }

    public Climber() {
        m_climberLeftMotor = new CANSparkMax(ClimberConstants.kClimberLeftCANID, MotorType.kBrushless);
        m_climberRightMotor = new CANSparkMax(ClimberConstants.kClimberRightCANID, MotorType.kBrushless);

       climberConfig(m_climberLeftMotor, true);
       climberConfig(m_climberRightMotor, false);

        setDefaultCommand();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
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

    /*
     * Defualt command for the climber - stops the climber motors
     */
    public void setDefaultCommand() {
        super.setDefaultCommand(Commands.run(this::stop, this));
    }

    public Command climbUp() {
        return Commands.run(() -> setDirection(ClimberDirection.UP), this);
    }

    public Command climbDown() {
        return Commands.run(() -> setDirection(ClimberDirection.DOWN), this);
    }
}
