package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeState;
import java.util.function.Supplier;

public class Intake extends SubsystemBase {

  private CANSparkMax m_intakeMotor;
  private CANSparkFlex m_feederMotor;
  private DigitalInput sensor;

  private IntakeState m_intakeState = IntakeState.IDLE;

  public Intake() {
    m_intakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorPort, CANSparkMax.MotorType.kBrushless);
    m_feederMotor = new CANSparkFlex(Constants.Intake.feederCANID, CANSparkMax.MotorType.kBrushless);
    sensor = new DigitalInput(Constants.Intake.kIntakeSensorPort);

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setSmartCurrentLimit(Constants.Intake.kIntakeCurrennLimit);
    m_intakeMotor.setInverted(Constants.Intake.kIntakeMotorInverted);
    m_intakeMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);
    m_intakeMotor.burnFlash();

    m_feederMotor.restoreFactoryDefaults();
    m_feederMotor.setSmartCurrentLimit(Constants.Intake.kFeederCurrentLimit);
    m_feederMotor.setInverted(Constants.Intake.kIntakeMotorInverted);
    m_feederMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);
    m_feederMotor.burnFlash();
  }

  @Override
  public void periodic() {
    System.out.println("Intake: " + m_intakeState);
    switch (m_intakeState) {
      case IDLE:
        idle();
        break;
      case INTAKE:
        intake();
        break;
      case OUTTAKE:
        outtake();
        break;
      case FIRE:
        spinFeeder();
        break;
      default:
        idle();
        break;
    }
  }
  
  public void idle()
  {
    m_intakeMotor.stopMotor();
    m_feederMotor.stopMotor();
  }

  public void intake()
  {
    m_intakeMotor.set(Constants.Intake.intakeSpeed);
    m_feederMotor.set(Constants.Intake.feederSpeed);
  }

  public void outtake()
  {
    m_intakeMotor.set(-Constants.Intake.intakeSpeed);
    m_feederMotor.set(-Constants.Intake.feederSpeed);
  }

  public void setState(IntakeState state)
  {
    m_intakeState = state;
  }

  public void spinFeeder()
  {
    m_feederMotor.set(1);
  }

  public void stopFeeder() {
    m_feederMotor.set(0);
  }

}
