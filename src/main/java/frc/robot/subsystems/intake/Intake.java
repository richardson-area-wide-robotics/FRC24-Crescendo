package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
    m_feederMotor.setInverted(false);
    m_feederMotor.setIdleMode(IdleMode.kBrake);
    m_feederMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // System.out.println("Intake: " + m_intakeState);
    // SmartDashboard.putBoolean("sensor enabled", sensorEnabled());
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

  public boolean sensorEnabled(){
    return sensor.get();
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

//   public Command receive() {
//     return commandBuilder()
//         .onInitialize(() -> feedMotor.set(FeederConstants.RECEIVE_SPEED))
//         .isFinished(() -> hasNote())
//         .onEnd(() -> feedMotor.stopMotor())
//         .onlyIf(() -> !hasNote())
//         .withName("feeder.receive()");
// }

  /* make a command that only runs when a button is pressed and then runs the feeder and intake together in the positive speed
   * and will stop the commande when the sensor is triggered or the button is released; using function command class
   */ 
  public Command receive() {
    return new FunctionalCommand(
      () -> {
        return;
      },
      () -> {
        m_feederMotor.set(Constants.Intake.feederSpeed);
        m_intakeMotor.set(Constants.Intake.intakeSpeed);;
      },
      (end) -> {
        m_feederMotor.stopMotor();
        m_intakeMotor.stopMotor();
      },
      () -> {
        // if(!sensor.get()) {return true;}   
        // else {return false;}
        return false;
      },
      this
    );
  }
}
