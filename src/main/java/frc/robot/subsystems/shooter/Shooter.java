package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkFlex m_kickerMotor;
    private CANSparkFlex m_leftShooterMotor;
    private CANSparkFlex m_rightShooterMotor;
    private CANSparkMax m_backIntakeMotor;
}
