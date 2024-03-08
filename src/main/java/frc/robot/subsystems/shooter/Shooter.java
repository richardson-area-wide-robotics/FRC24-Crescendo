package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ShooterState;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex m_kickerMotor;
    private final CANSparkFlex m_shooterLeftMotor;
    private final CANSparkFlex m_shooterRightMotor;

    private SparkPIDController kickerPIDController;
    private SparkPIDController shooterLeftPIDController;
    private SparkPIDController shooterRightPIDController;

    private RelativeEncoder kickerEncoder;
    private RelativeEncoder shooterLeftEncoder;
    private RelativeEncoder shooterRightEncoder;

    private ShooterState m_shooterState;

    private final SimpleMotorFeedforward feedforward;
    private double leftTargetSpeed = 0.0;
    private double rightTargetSpeed = 0.0;
    private boolean leftPIDActive = false;
    private boolean rightPIDActive = false;

    public void kickerConfig(CANSparkFlex motor){
        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(ShooterConstants.kKickerMotorCurrentLimit);

        motor.setIdleMode(ShooterConstants.kKickerMotorIdleMode);

        motor.setInverted(ShooterConstants.kKickerMotorInvert);

        kickerPIDController = motor.getPIDController();

        kickerEncoder = motor.getEncoder();

        m_shooterState = ShooterState.IDLE;
        
    }

    public void shooterConfig(CANSparkFlex motor, boolean shooterLeftSide){
        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(ShooterConstants.kShooterMotorCurrentLimit);

        motor.setIdleMode(ShooterConstants.kShooterMotorIdleMode);

        if(shooterLeftSide) {
            motor.setInverted(ShooterConstants.kShooterLeftMotorInverted);
            shooterLeftPIDController = motor.getPIDController();
            shooterLeftEncoder = motor.getEncoder();
            shooterLeftPIDController.setP(ShooterConstants.PVal);
            shooterLeftEncoder.setPositionConversionFactor(ShooterConstants.REL_ENC_CONVERSION);
            shooterLeftEncoder.setVelocityConversionFactor(ShooterConstants.REL_ENC_CONVERSION);
        }

        if (!shooterLeftSide) {
            motor.setInverted(ShooterConstants.kShooterRightMotorInverted);
            shooterRightPIDController = motor.getPIDController();
            shooterRightEncoder = motor.getEncoder();
            shooterRightPIDController.setP(ShooterConstants.PVal);
            shooterRightEncoder.setPositionConversionFactor(ShooterConstants.REL_ENC_CONVERSION);
            shooterRightEncoder.setVelocityConversionFactor(ShooterConstants.REL_ENC_CONVERSION);
        }
    }

    public Shooter() {
        m_kickerMotor = new CANSparkFlex(ShooterConstants.kKickerMotorCANID, MotorType.kBrushless);
        m_shooterLeftMotor = new CANSparkFlex(ShooterConstants.kShooterLeftCANID, MotorType.kBrushless);
        m_shooterRightMotor = new CANSparkFlex(ShooterConstants.kShooterRightCANID, MotorType.kBrushless);

        kickerConfig(m_kickerMotor);
        shooterConfig(m_shooterLeftMotor, true);
        shooterConfig(m_shooterRightMotor, false);

        feedforward =
        new SimpleMotorFeedforward(
            ShooterConstants.FEED_FORWARDKS,
            ShooterConstants.FEED_FORWARDKV,
            ShooterConstants.FEED_FORWARDKA
        );

        m_kickerMotor.burnFlash();
        m_shooterLeftMotor.burnFlash();
        m_shooterRightMotor.burnFlash();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Applied output", getSpeed());
        switch (m_shooterState) {
            case INTAKE:
                intake();
                break;
            case OUTTAKE:
                outtake();
                break;
            case SPEAKER:
                // Commands.run(()-> speakerMode());
                speakerMode();
                break;
            case AMP:
                ampSpeed();
                break;
            case REVERSE:
                reverse();
            case IDLE:
                idle();
                break;
        }
    }


    public void reverse() {
        // TODO: change to constants
        m_shooterState = ShooterState.REVERSE;
        m_shooterLeftMotor.set(-0.1);
        m_shooterRightMotor.set(-0.1);
    }

        public void toggleState(ShooterState state) {
        if (m_shooterState == state) {
            m_shooterState = ShooterState.IDLE;
        } else {
            m_shooterState = state;
        }
    }

    public void setStateSpeaker(ShooterState state){
        m_shooterState = state;
    }

    public void idle() {
        // if(speakerMode) {
        //   speakerMode();
        // } else if(ampMode) {
        //   ampMode();
        // } else {
        // stopAll();
        // }
        // m_shooterLeftMotor.stopMotor();
        // m_shooterRightMotor.stopMotor();
        m_shooterState = ShooterState.IDLE;
        m_shooterLeftMotor.set(0.1);;
        m_shooterRightMotor.set(0.1);
        m_kickerMotor.set(0.1);
    }

    public double getSpeed(){
       return m_shooterLeftMotor.getAppliedOutput();
    }

    public void stopAll() {
        m_shooterState = ShooterState.IDLE;
        m_kickerMotor.stopMotor();
        m_shooterLeftMotor.stopMotor();
        m_shooterRightMotor.stopMotor();
        // m_pivotRightMotor.stopMotor();
        // m_pivotLeftMotor.stopMotor();
    }

    public void intake() {
        m_shooterState = ShooterState.INTAKE;
    }

    public void outtake() {
        m_shooterState = ShooterState.OUTTAKE;
    }

    private void ampSpeed() {
        m_shooterLeftMotor.set(0.1);
        m_shooterRightMotor.set(0.1);
        m_kickerMotor.set(0.1);
    }

     /**
     * While called, angles the pivot of the shooter and sets the shooter to the output speed neccissary
     * to score from the bot's distance from the shooter. However, does not shoot the note. 
     */
    private void speakerMode() {
        m_shooterLeftMotor.set(0.6);
        m_shooterRightMotor.set(0.4);
        m_kickerMotor.set(0.75); // TODO: change to constant

        // applySpeed(1.0);
    }

    public void applySpeed(double speed){
        double leftSpeed = speed;
        double rightSpeed = speed * ShooterConstants.RIGHT_PERCENT_OF_LEFT;

        if (speed == 0.0) {
            m_shooterLeftMotor.stopMotor();
            m_shooterRightMotor.stopMotor();
            leftPIDActive = false;
            rightPIDActive = false;
        } else {
            double leftDelta = leftSpeed - shooterLeftEncoder.getVelocity();
            if (Math.abs(leftDelta) < ShooterConstants.PID_ACTIVE_RANGE) {
                shooterLeftPIDController.setReference(leftSpeed, ControlType.kVelocity, 0, feedforward.calculate(leftSpeed));
                leftPIDActive = true;
            } else {
                m_shooterLeftMotor.set(ShooterConstants.RAMP_SPEED * Math.signum(leftDelta));
                leftPIDActive = false;
            }

            double rightDelta = rightSpeed - shooterRightEncoder.getVelocity();
            if (Math.abs(rightDelta) < ShooterConstants.PID_ACTIVE_RANGE) {
                shooterRightPIDController.setReference(rightSpeed, ControlType.kVelocity, 0, feedforward.calculate(rightSpeed));
                rightPIDActive = true;
            } else {
                m_shooterRightMotor.set(ShooterConstants.RAMP_SPEED * Math.signum(rightDelta));
                rightPIDActive = false;
            }
        }

        leftTargetSpeed = leftSpeed;
        rightTargetSpeed = rightSpeed;
    }

    
}
