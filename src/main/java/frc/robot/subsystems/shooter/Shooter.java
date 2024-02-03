package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_feederMotor;
    private CANSparkFlex m_kickerMotor;
    private CANSparkFlex m_shooterLeftMotor;
    private CANSparkFlex m_shooterRightMotor;

    private SparkPIDController m_shooterLeftPIDController;
    private SparkPIDController m_shooterRightPIDController;

    // TODO: constructor
    public Shooter() {
        m_feederMotor = new CANSparkMax(Constants.ShooterConstants.feederMotorCANID, MotorType.kBrushless);
        m_kickerMotor = new CANSparkFlex(Constants.ShooterConstants.kickerMotorCANID, MotorType.kBrushless);
        m_shooterLeftMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLeftCANID, MotorType.kBrushless);
        m_shooterRightMotor = new CANSparkFlex(Constants.ShooterConstants.shooterRightCANID, MotorType.kBrushless);

        m_feederMotor.restoreFactoryDefaults();
        m_kickerMotor.restoreFactoryDefaults();
        m_shooterLeftMotor .restoreFactoryDefaults();
        m_shooterRightMotor.restoreFactoryDefaults();

        m_shooterLeftPIDController = m_shooterLeftMotor.getPIDController();
        m_shooterRightPIDController = m_shooterRightMotor.getPIDController();

        // Set PID values
    }

    // TODO: spinFeeder
    private void spinFeeder(double speed)
    {
       m_feederMotor.set(speed);
    }

    // TODO: spinKicker
    private void spinKicker() {

    }

    // TODO: spinShooter
    private void spinShooterAngular(Measure<Velocity<Angle>> leftSpeed, Measure<Velocity<Angle>> rightSpeed) {
        m_shooterLeftPIDController.setReference(leftSpeed.in(RPM), CANSparkFlex.ControlType.kVelocity);
        m_shooterRightPIDController.setReference(rightSpeed.in(RPM), CANSparkFlex.ControlType.kVelocity);
    }

    /**
     * Spins shooter wheels such that the tangential velocity at a point along the circumfrence is the velocity given in meters per second.
     * @param leftLinear
     * @param rightLinear
     */
    private void spinShooterLinear(Measure<Velocity<Distance>> leftLinear, Measure<Velocity<Distance>> rightLinear) {
        Measure<Velocity<Angle>> leftAngular = wheelSpeedToRotation(leftLinear, Constants.ShooterConstants.shooterWheelRadius);
        Measure<Velocity<Angle>> rightAngular = wheelSpeedToRotation(rightLinear, Constants.ShooterConstants.shooterWheelRadius);
        spinShooterAngular(leftAngular,rightAngular);
    }

    public Measure<Velocity<Angle>> wheelSpeedToRotation(Measure<Velocity<Distance>> speed, Measure<Distance> radius)
    {
       return RadiansPerSecond.of(speed.in(MetersPerSecond)/radius.in(Meters));
    }

    public Measure<Velocity<Distance>> wheelRotationToSpeed(Measure<Velocity<Angle>> speed, Measure<Distance> radius)
    {
       return MetersPerSecond.of(speed.in(RadiansPerSecond)*radius.in(Meters));
    }

    // TODO: stop method
    public void stopAll() {
      m_feederMotor.stopMotor();
      m_kickerMotor.stopMotor();
      m_shooterLeftMotor.stopMotor();
      m_shooterRightMotor.stopMotor();
    }

    // TODO: speaker shot method
    public void setShootSpeed(Measure<Velocity<Distance>> launchSpeed, Measure<Velocity<Angle>> rotationalSpeed)
    {
        double num = wheelRotationToSpeed(rotationalSpeed, Constants.ShooterConstants.shooterWheelRadius).in(MetersPerSecond);
        Measure<Velocity<Distance>> leftSpeed = MetersPerSecond.of(launchSpeed.in(MetersPerSecond)-num);
        Measure<Velocity<Distance>> rightSpeed = MetersPerSecond.of(launchSpeed.in(MetersPerSecond)+num);
        spinShooterLinear(leftSpeed,rightSpeed);
    }
    

    // TODO: getIsAtShooterSpeed method
    private boolean getIsAtShooterSpeed() {
        // TODO
        
        return true;
    }


    // TODO: speakerFire method
    public void speakerFire() {
        
    }

}
