package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

private CANSparkMax m_intakeMotor;
private DigitalInput sensor;

public Intake(){
    m_intakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorPort, CANSparkMax.MotorType.kBrushless);
    sensor = new DigitalInput(Constants.Intake.kIntakeSensorPort);

    m_intakeMotor.restoreFactoryDefaults();

    m_intakeMotor.setSmartCurrentLimit(Constants.Intake.kIntakeCurrennLimit);
    m_intakeMotor.setInverted(Constants.Intake.kIntakeMotorInverted);
    m_intakeMotor.setIdleMode(Constants.Intake.kIntakeIdleMode);

    m_intakeMotor.burnFlash();
}

}
