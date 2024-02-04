package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

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

    private AbsoluteEncoder pivotAngle;
    private RelativeEncoder leftShotSpeed;
    private RelativeEncoder rightShotSpeed;

    private SparkPIDController m_feederPIDController;
    private SparkPIDController m_kickerPIDController;
    private SparkPIDController m_shooterLeftPIDController;
    private SparkPIDController m_shooterRightPIDController;

    private double desiredPivot;
    private Measure<Velocity<Distance>> desiredShotSpeed;
    private Measure<Velocity<Angle>> desiredRotation; 

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
        m_kickerPIDController = m_kickerMotor.getPIDController();
        m_feederPIDController = m_feederMotor.getPIDController();
  
        rightShotSpeed = m_shooterRightMotor.getEncoder();
        leftShotSpeed = m_shooterLeftMotor.getEncoder();
        leftShotSpeed.setVelocityConversionFactor(1/(120*Math.PI));
        rightShotSpeed.setVelocityConversionFactor(1/(120*Math.PI));
        pivotAngle.setPositionConversionFactor(360);

        desiredPivot = 0;
        desiredShotSpeed = MetersPerSecond.of(0.0);    
        desiredRotation = RadiansPerSecond.of(0.0);
        // Set PID values
    }

    // TODO: spinFeeder
    private void spinFeeder(Measure<Velocity<Angle>> speed){
       m_feederPIDController.setReference(speed.in(RadiansPerSecond),CANSparkFlex.ControlType.kVelocity);
    }

    // TODO: spinKicker
    private void spinKicker(Measure<Velocity<Angle>> speed){
       m_kickerPIDController.setReference(speed.in(RadiansPerSecond),CANSparkFlex.ControlType.kVelocity);
    }

    // TODO: spinShooter
    private void spinShooterAngular(Measure<Velocity<Angle>> leftSpeed, Measure<Velocity<Angle>> rightSpeed) {
        m_shooterLeftPIDController.setReference(leftSpeed.in(RPM), CANSparkFlex.ControlType.kVelocity);
        m_shooterRightPIDController.setReference(rightSpeed.in(RPM), CANSparkFlex.ControlType.kVelocity);
    }

    /**
     * Spins shooter wheels such that the tangential velocity at a point along the circumference is the velocity given in meters per second.
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

    public void stopAll() {
      m_feederMotor.stopMotor();
      m_kickerMotor.stopMotor();
      m_shooterLeftMotor.stopMotor();
      m_shooterRightMotor.stopMotor();
    }

    // TODO: speaker shot method
    public void setShootSpeed(Measure<Velocity<Distance>> launchSpeed, Measure<Velocity<Angle>> rotationalSpeed)
    {
        double num = wheelRotationToSpeed(rotationalSpeed, Inches.of(12)).in(MetersPerSecond);
        Measure<Velocity<Distance>> leftSpeed = MetersPerSecond.of(launchSpeed.in(MetersPerSecond)-num);
        Measure<Velocity<Distance>> rightSpeed = MetersPerSecond.of(launchSpeed.in(MetersPerSecond)+num);
        spinShooterLinear(leftSpeed,rightSpeed);
    }
    

    
    public void calcShotSpeed()
    {
      desiredShotSpeed = MetersPerSecond.of(4.0);
    }

    /**
     * Calculates the pivot of the shooter required to shoot in the speaker from the robot's current position.
     * @return
     */
    public double calcPivot()
    {
        return 0.0;
    }

    public void speakerMode()
    {

    }

    /**
     * Returns true if the current position of the pivot of the shooter is within the tolerance of the desired pivot angle.
     * Tolerance in degrees is defined in ShooterConstants.
     * @return
     */
    private boolean getIsAtDesiredPivot() {
        double pos = pivotAngle.getPosition();
        double tol = Constants.ShooterConstants.pivotToleranceDegrees;
        if (pos <= desiredPivot + tol && pos >= desiredPivot - tol)
        {
          return true;
        }
        return false;
    }

    private boolean getIsAtLeftShooterSpeed() {
        double desired = desiredShotSpeed.in(MetersPerSecond)-wheelRotationToSpeed(desiredRotation, Inches.of(12)).in(MetersPerSecond);
        double tol = Constants.ShooterConstants.launchSpeedTolerance.in(MetersPerSecond);
        double speed = wheelRotationToSpeed(RadiansPerSecond.of(leftShotSpeed.getVelocity()),Constants.ShooterConstants.shooterWheelRadius).in(MetersPerSecond);
        if (speed <= desired + tol && speed >= desired - tol)
        {
          return true;
        }
        return false;
    }

    private boolean getIsAtRightShooterSpeed() {
        double desired = desiredShotSpeed.in(MetersPerSecond)+wheelRotationToSpeed(desiredRotation, Inches.of(12)).in(MetersPerSecond);
        double tol = Constants.ShooterConstants.launchSpeedTolerance.in(MetersPerSecond);
        double speed = wheelRotationToSpeed(RadiansPerSecond.of(rightShotSpeed.getVelocity()),Constants.ShooterConstants.shooterWheelRadius).in(MetersPerSecond);
        if (speed <= desired + tol && speed >= desired - tol)
        {
          return true;
        }
        return false;
    }

    private boolean getIsAtShooterSpeed() {
        if(getIsAtLeftShooterSpeed() && getIsAtRightShooterSpeed())
        {
            return true;
        }
        return false;
    }

    // TODO: speakerFire method
    public void speakerFire() {
        if(getIsAtShooterSpeed() && getIsAtDesiredPivot())
        {
            spinKicker(Constants.ShooterConstants.kickerSpeed);
        }
    }

}
