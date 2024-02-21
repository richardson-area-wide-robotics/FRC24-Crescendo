package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax m_intakeMotor;
  private CANSparkFlex m_feederMotor;
  private DigitalInput sensor;

  public Intake() {
    m_intakeMotor = new CANSparkMax(Constants.Intake.intakeCANID, CANSparkMax.MotorType.kBrushless);
    m_feederMotor = new CANSparkFlex(Constants.Intake.feederCANID, CANSparkMax.MotorType.kBrushless);
    sensor = new DigitalInput(Constants.Intake.kIntakeSensorPort);

    m_intakeMotor.restoreFactoryDefaults();
    m_feederMotor.restoreFactoryDefaults();

    m_intakeMotor.setSmartCurrentLimit(Constants.Intake.kIntakeCurrennLimit);
    m_intakeMotor.setInverted(Constants.Intake.kIntakeMotorInverted);
    m_intakeMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);

    m_feederMotor.setSmartCurrentLimit(Constants.Intake.kFeederCurrentLimit);
    m_feederMotor.setInverted(Constants.Intake.kIntakeMotorInverted);
    m_feederMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);

    m_feederMotor.burnFlash();
    m_intakeMotor.burnFlash();
  }
  
  public void idle()
  {
    m_intakeMotor.stopMotor();
    m_feederMotor.stopMotor();
  }

  public void intake()
  {
    m_intakeMotor.set(-Constants.Intake.intakeSpeed);
    m_feederMotor.set(Constants.Intake.feederSpeed);
  }

  public void outtake()
  {
    m_intakeMotor.set(Constants.Intake.intakeSpeed);
    m_feederMotor.set(-Constants.Intake.feederSpeed);
  }

  public void spinFeeder()
  {
    m_feederMotor.set(1);
  }

}
