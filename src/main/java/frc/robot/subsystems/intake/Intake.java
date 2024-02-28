package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_intakeMotor;

    private IntakeState m_intakeState = IntakeState.IDLE;

    private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    private GenericEntry m_speedEntry = intakeTab.add("Intake Speed", 0).getEntry();
    private GenericEntry m_currentEntry = intakeTab.add("Intake Current", 0).getEntry();
    private GenericEntry m_stateEntry = intakeTab.add("Intake State", m_intakeState).getEntry();

    public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeCANID, CANSparkMax.MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
    m_intakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);
    m_intakeMotor.setIdleMode(IntakeConstants.kIntakeIdleMode);
    m_intakeMotor.burnFlash();

    this.setDefaultCommand();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_speedEntry.setDouble(getSpeed());
        m_currentEntry.setDouble(getCurrent());

        // switch (m_intakeState) {
        //     case IDLE:
        //       idle();
        //       break;
        //     case INTAKE:
        //       intake();
        //       break;
        //     case OUTTAKE:
        //       outtake();
        //       break;
        //     default:
        //       break;
        //   }

        m_stateEntry.setString(m_intakeState.toString());
    }

    /**
     * Returns the output current of the motor
     */
    public double getCurrent() {
        return m_intakeMotor.getOutputCurrent();
    }

    /**
     * Returns the speed of the motor
     */
    public double getSpeed() {
        return m_intakeMotor.get();
    }

    /*
     * Stop the intake motor
     */
    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    /*
     * Commands the intake to spin in the positive direction - intaking
     */
    public Command intake() {
        return Commands.run(()-> m_intakeMotor.set(IntakeConstants.kIntakeSpeed), this);
    }

    /*
     * Commands the intake to spin in the negative direction - outtaking
     */
    public Command outtake() {
        return Commands.run(()-> m_intakeMotor.set(-IntakeConstants.kIntakeSpeed), this);
    }

    /*
     * Set the default Command for the subsystem
     */
    public void setDefaultCommand() {
        setDefaultCommand(new RunCommand(this::stopIntake, this));
    }
    
}
