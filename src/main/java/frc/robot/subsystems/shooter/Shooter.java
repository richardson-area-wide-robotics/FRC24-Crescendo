package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class Shooter extends SubsystemBase {
    private CANSparkFlex m_kickerMotor;
    private CANSparkFlex m_shooterLeftMotor;
    private CANSparkFlex m_shooterRightMotor;

    private CANSparkMax m_pivotLeftMotor;
    private CANSparkMax m_pivotRightMotor;

    private AbsoluteEncoder m_pivotEncoder;
    private RelativeEncoder m_shooterLeftEncoder;
    private RelativeEncoder m_shooterRightEncoder;

    private boolean speakerMode;
    private boolean ampMode;
    private boolean firing;
    private boolean reverseShooterWheels;

    private SparkPIDController m_kickerPIDController;
    private SparkPIDController m_shooterLeftPIDController;
    private SparkPIDController m_shooterRightPIDController;
    private SparkPIDController m_pivotPIDController;

    private Measure<Angle> desiredPivotAngle;
    private Measure<Velocity<Distance>> desiredShotSpeed;
    private Measure<Velocity<Angle>> desiredRotationSpeed;

    public Shooter() {
        m_kickerMotor = new CANSparkFlex(Constants.ShooterConstants.kickerMotorCANID, MotorType.kBrushless);
        m_shooterLeftMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLeftCANID, MotorType.kBrushless);
        m_shooterRightMotor = new CANSparkFlex(Constants.ShooterConstants.shooterRightCANID, MotorType.kBrushless);
        m_pivotLeftMotor = new CANSparkMax(Constants.ShooterConstants.pivotLeftCANID, MotorType.kBrushless);
        m_pivotRightMotor = new CANSparkMax(Constants.ShooterConstants.pivotRightCANID, MotorType.kBrushless);

        m_kickerMotor.restoreFactoryDefaults();
        m_kickerMotor.setSmartCurrentLimit(Constants.ShooterConstants.kickerMotorCurrentLimit);
        m_kickerMotor.burnFlash();

        m_shooterLeftMotor.restoreFactoryDefaults();
        m_shooterLeftMotor.setIdleMode(IdleMode.kCoast);
        m_shooterLeftMotor.setInverted(true); // TODO: change to constant
        m_shooterLeftMotor.setSmartCurrentLimit(Constants.ShooterConstants.shooterLeftMotorCurrentLimit);
        m_shooterLeftMotor.burnFlash();

        m_shooterRightMotor.restoreFactoryDefaults();
        m_shooterRightMotor.setIdleMode(IdleMode.kCoast);
        m_shooterRightMotor.setInverted(false); // TODO: change to constant
        m_shooterRightMotor.setSmartCurrentLimit(Constants.ShooterConstants.shooterRightMotorCurrentLimit);
        m_shooterRightMotor.burnFlash();

        m_pivotLeftMotor.restoreFactoryDefaults();
        m_pivotLeftMotor.setSmartCurrentLimit(Constants.ShooterConstants.pivotLeftMotorCurrentLimit);
        m_pivotLeftMotor.follow(m_pivotRightMotor, true);
        m_pivotLeftMotor.burnFlash();

        m_pivotRightMotor.restoreFactoryDefaults();
        m_pivotRightMotor.setIdleMode(IdleMode.kBrake);
        m_pivotRightMotor.setInverted(Constants.ShooterConstants.pivotRightMotorInverted);
        m_pivotRightMotor.setSmartCurrentLimit(Constants.ShooterConstants.pivotRightMotorCurrentLimit);
        m_pivotRightMotor.burnFlash();



        m_shooterLeftPIDController = m_shooterLeftMotor.getPIDController();
        m_shooterRightPIDController = m_shooterRightMotor.getPIDController();
        m_kickerPIDController = m_kickerMotor.getPIDController();
        m_pivotPIDController = m_pivotRightMotor.getPIDController();

        m_shooterRightEncoder = m_shooterRightMotor.getEncoder();
        m_shooterLeftEncoder = m_shooterLeftMotor.getEncoder();
        m_pivotEncoder = m_pivotRightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        desiredPivotAngle = Degrees.of(0);
        desiredShotSpeed = MetersPerSecond.of(0.0);
        desiredRotationSpeed = RadiansPerSecond.of(0.0);

        speakerMode = false;
        firing = false;
        reverseShooterWheels = false;

        setPIDValues();

    }

    private void spinKicker(Measure<Velocity<Angle>> speed) {
        m_kickerPIDController.setReference(speed.in(RPM), CANSparkFlex.ControlType.kVelocity);
    }

    private void spinShooterAngular(Measure<Velocity<Angle>> leftSpeed, Measure<Velocity<Angle>> rightSpeed) {
        m_shooterLeftPIDController.setReference(leftSpeed.in(RPM), CANSparkFlex.ControlType.kVelocity);
        m_shooterRightPIDController.setReference(rightSpeed.in(RPM), CANSparkFlex.ControlType.kVelocity);
    }

    private void pivotSpeed(double speedPercentage) {
        m_pivotRightMotor.set(speedPercentage);
    }

    /** 
     * Pivots the shooter to a given angle about the axis of the absolute encoder. 
     */
    private void pivotTo(Measure<Angle> angle) {
        m_pivotPIDController.setReference(angle.in(Rotations), CANSparkFlex.ControlType.kPosition);
    }

    public void pivotTo(double angle) {
        m_pivotPIDController.setReference(angle, CANSparkFlex.ControlType.kPosition);
    }


    /**
     * Spins shooter wheels such that the tangential velocity at a point along the
     * circumference is the velocity given in meters per second.
     * 
     * @param leftLinear
     * @param rightLinear
     */
    private void spinShooterLinear(Measure<Velocity<Distance>> leftLinear, Measure<Velocity<Distance>> rightLinear) {
        Measure<Velocity<Angle>> leftAngular = wheelSpeedToRotation(leftLinear,
                Constants.ShooterConstants.kShooterWheelRadius);
        Measure<Velocity<Angle>> rightAngular = wheelSpeedToRotation(rightLinear,
                Constants.ShooterConstants.kShooterWheelRadius);
        spinShooterAngular(leftAngular, rightAngular);
    }

    /**
     * converts a linear velocity to a rotational speed based on the provided
     * radius.
     * 
     * @param speed
     * @param radius
     * @return
     */
    public Measure<Velocity<Angle>> wheelSpeedToRotation(Measure<Velocity<Distance>> speed, Measure<Distance> radius) {
        return RadiansPerSecond.of(speed.in(MetersPerSecond) / radius.in(Meters));
    }
    
    private void setPIDValues()
    {
        m_pivotPIDController.setFeedbackDevice(m_pivotEncoder);
        m_pivotPIDController.setPositionPIDWrappingEnabled(false);
        m_pivotPIDController.setP(Constants.ShooterConstants.kPivotP);
        m_pivotPIDController.setI(Constants.ShooterConstants.kPivotI);
        m_pivotPIDController.setD(Constants.ShooterConstants.kPivotD);

        m_shooterRightPIDController.setP(Constants.ShooterConstants.kShooterP);
        m_shooterRightPIDController.setI(Constants.ShooterConstants.kShooterI);
        m_shooterRightPIDController.setD(Constants.ShooterConstants.kShooterD);

        m_shooterLeftPIDController.setP(Constants.ShooterConstants.kShooterP);
        m_shooterLeftPIDController.setI(Constants.ShooterConstants.kShooterI);
        m_shooterLeftPIDController.setD(Constants.ShooterConstants.kShooterD);

        m_kickerPIDController.setP(Constants.ShooterConstants.kPivotP);
        m_kickerPIDController.setI(Constants.ShooterConstants.kPivotI);
        m_kickerPIDController.setD(Constants.ShooterConstants.kPivotD);
    }
    

    /**
     * converts a rotational speed to a linear velocity based on the provided
     * radius.
     * @param speed
     * @param radius
     * @return
     */
    public Measure<Velocity<Distance>> wheelRotationToSpeed(Measure<Velocity<Angle>> speed, Measure<Distance> radius) {
        return MetersPerSecond.of(speed.in(RadiansPerSecond) * radius.in(Meters));
    }

    public void idle() {
        // if(speakerMode) {
        //   speakerMode();
        // } else if(ampMode) {
        //   ampMode();
        // } else {
        // stopAll();
        // }
        m_pivotLeftMotor.stopMotor();
        m_pivotRightMotor.stopMotor();
    }

    public void stopAll() {
        m_kickerMotor.stopMotor();
        m_shooterLeftMotor.stopMotor();
        m_shooterRightMotor.stopMotor();
        m_pivotRightMotor.stopMotor();
        m_pivotLeftMotor.stopMotor();
    }

    /**
     * Spins the shooter wheels based on a given launch speed and rotational speed of the note.
     */
    public void setShootSpeed(Measure<Velocity<Distance>> launchSpeed, Measure<Velocity<Angle>> rotationalSpeed) {
        //num represents the change in the linear speed required to rotate the note at the provided rotationalSpeed.
        double num = wheelRotationToSpeed(rotationalSpeed, Inches.of(12)).in(MetersPerSecond);
        Measure<Velocity<Distance>> leftSpeed = MetersPerSecond.of(launchSpeed.in(MetersPerSecond) - num);
        Measure<Velocity<Distance>> rightSpeed = MetersPerSecond.of(launchSpeed.in(MetersPerSecond) + num);
        spinShooterLinear(leftSpeed, rightSpeed);
    }


    /**
     * Calculates the output speed of the shooter required to shoot in the speaker from the
     * robot's current position.
     * 
     * Should take in the robots position on the feild given by Swerve Odometry.
     */
    public void calcShotSpeed() {
        desiredShotSpeed = MetersPerSecond.of(4);
        System.out.println(desiredShotSpeed);
        System.out.println("Rotations per second: "+4/Math.pow(0.05*Math.PI,2));
        System.out.println("Actual RPM: "+m_shooterRightEncoder.getVelocity());
        System.out.println("Expected RPM: "+4/(0.05*Math.PI*2)*60);
    }

    /**
     * Calculates the pivot of the shooter required to shoot in the speaker from the
     * robot's current position.
     * 
     * Should take in the robots position on the field given by Swerve Odometry.
     * @return
     */
    public void calcPivot() {
        desiredPivotAngle = Degrees.of(45);
        System.out.println(desiredPivotAngle);
    }

    public void pivot(Constants.ShooterConstants.PivotDirection direction) {
        switch (direction) {
            case UP:
                pivotSpeed(Constants.ShooterConstants.kPivotSpeed);
                break;
            case DOWN:
                pivotSpeed(-Constants.ShooterConstants.kPivotSpeed);
                break;
            default:
                pivotSpeed(0);
        }
    }

    public void toggleAmpMode()
    {
        ampMode = true;
        speakerMode = false;
    }

    public void toggleSpeakerMode()
    {
        // speakerMode = true;
        // ampMode = false;

        speakerMode = !speakerMode;
        if (speakerMode) {
            speakerMode();
        } else {
            stopAll();
        }
    }

    public void toggleOff()
    {
        ampMode = false;
        speakerMode = false;
    }
    
    /**
     * While called, angles the pivot of the shooter and sets the shooter to the output speed neccissary
     * to score from the bot's distance from the shooter. However, does not shoot the note. 
     */
    private void speakerMode() {
        // calcPivot();
        // calcShotSpeed();
        // pivotTo(desiredPivotAngle);
        // //System.out.println(m_shooterLeftEncoder.getVelocity());
        // //System.out.println(m_shooterRightEncoder.getVelocity());
        // // m_shooterRightPIDController.setReference(Math.pow(Math.PI*2,2)*desiredShotSpeed.in(MetersPerSecond), CANSparkFlex.ControlType.kVelocity);
        // //m_shooterRightPIDController.setReference(wheelSpeedToRotation(desiredShotSpeed,Inches.of(2)).in(RPM), CANSparkFlex.ControlType.kVelocity);
        // spinKicker(RPM.of(240));
        // setShootSpeed(desiredShotSpeed, desiredRotationSpeed);

        setLeftShooterSpeed(1);
        setRightShooterSpeed(0.9);
        m_kickerMotor.set(0.75); // TODO: change to constant
    }
    
    private void ampMode()
    {
        pivotTo(Degrees.of(Constants.ShooterConstants.Pivot.ampAngleDegrees));
        m_shooterRightPIDController.setReference(-240,ControlType.kVelocity);
        m_shooterLeftPIDController.setReference(-240,ControlType.kVelocity);
    }

    /**
     * Returns true if the current position of the pivot of the shooter is within
     * the tolerance of the desired pivot angle.
     * Tolerance in degrees is defined in Constants.ShooterConstants.
     * 
     * @return
     */
    private boolean getIsAtDesiredPivotAngle() {
        Measure<Angle> currentAngle = Rotations.of(m_pivotEncoder.getPosition());
        Measure<Angle> toleranceAngle = Constants.ShooterConstants.Pivot.toleranceAngle;
        
        return Math.abs(currentAngle.in(Degrees) - desiredPivotAngle.in(Degrees)) <= toleranceAngle.in(Degrees);
    }

    private boolean getIsAtLeftShooterSpeed() {
        Measure<Velocity<Distance>> desiredWheelSpeedLinear = 
            desiredShotSpeed.minus(wheelRotationToSpeed(desiredRotationSpeed, Constants.ShooterConstants.kHorizontalNoteCompression));
        Measure<Velocity<Distance>> currentWheelSpeedLinear =
            wheelRotationToSpeed(RPM.of(m_shooterLeftEncoder.getVelocity()), Constants.ShooterConstants.kShooterWheelRadius);

        return Math.abs(desiredWheelSpeedLinear.minus(currentWheelSpeedLinear).in(MetersPerSecond)) 
            <= Constants.ShooterConstants.kLaunchSpeedTolerance.in(MetersPerSecond);
    }

    private boolean getIsAtRightShooterSpeed() {
        Measure<Velocity<Distance>> desiredWheelSpeedLinear = 
            desiredShotSpeed.plus(wheelRotationToSpeed(desiredRotationSpeed, Constants.ShooterConstants.kHorizontalNoteCompression));
        Measure<Velocity<Distance>> currentWheelSpeedLinear =
            wheelRotationToSpeed(RPM.of(m_shooterLeftEncoder.getVelocity()), Constants.ShooterConstants.kShooterWheelRadius);

        return Math.abs(desiredWheelSpeedLinear.minus(currentWheelSpeedLinear).in(MetersPerSecond)) 
            <= Constants.ShooterConstants.kLaunchSpeedTolerance.in(MetersPerSecond);
    }

    private boolean getIsAtShooterSpeed() {
        return getIsAtLeftShooterSpeed() && getIsAtRightShooterSpeed();
    }

    /**
     * Spins the kicker enough to fire only when the shooter is at the neccissary launch speed and pivot angle.
     */
    public void fire(Intake m_intake) {
        // spinKicker(RPM.of(240));
        // firing = !firing;
        // if (firing) {
            m_intake.spinFeeder();
        // } else {
        //     m_intake.stopFeeder();
        // }
    }

    public Measure<Angle> getCurrentPivotAngle() {
        return Rotations.of(m_pivotEncoder.getPosition());
    }

    public void setLeftShooterSpeed(double speed) {
        m_shooterLeftMotor.set(speed);
    }

    public void setRightShooterSpeed(double speed) {
        m_shooterRightMotor.set(speed);
    }

    public void setFiring(boolean value) {
        firing = value;
    }

    public boolean getFiring() {
        return firing;
    }

    public void toggleReverseShooterWheels() {
        reverseShooterWheels = !reverseShooterWheels;

        if (reverseShooterWheels) {
            m_shooterLeftMotor.set(-0.1);
            m_shooterRightMotor.set(-0.1);
        } else {
            m_shooterLeftMotor.set(0);
            m_shooterRightMotor.set(0);
        }
    }
}
