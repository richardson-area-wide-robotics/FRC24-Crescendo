package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.drive.DriveSubsystem;



public class Lock extends Command{

    DriveSubsystem drive;
    DoubleSupplier sideways;
    DoubleSupplier forward;
    Camera camera;

    //PID controller for yawRate
    final PIDController yawRateController = new PIDController(
        Constants.ModuleConstants.kVisionTurningPIDGains.P,
        Constants.ModuleConstants.kVisionTurningPIDGains.I,
        Constants.ModuleConstants.kVisionTurningPIDGains.D);

    /**
     *
     @param drive passes controller
     @param camera passes camera
     @param forward passes y translation
     @param sideways passes x translation
    */
    public Lock(DriveSubsystem drive, Camera camera, DoubleSupplier forward, DoubleSupplier sideways ) {
        this.drive = drive;
        this.forward = forward;
        this.sideways = sideways;
        this.camera = camera;
        this.addRequirements(camera, drive);
    }
    
    
    @Override
    public void initialize() {
        
        yawRateController.reset();
    }  

    @Override
    public boolean isFinished() {
        return false;
    }
        
    //This sets the yawRate to circle the desired object while maintaning driver controll of motion
    @Override
    public  void execute() {
        double angularOffset = camera.getAngle();
        double yawRate = yawRateController.calculate(angularOffset, 0);

        //limits robot max speed while in locked-on mode
        double limitedForward = Math.max(forward.getAsDouble(), -1.0 * Constants.ModuleConstants.MAX_LOCKED_ON_SPEED);
        limitedForward = Math.min(limitedForward, Constants.ModuleConstants.MAX_LOCKED_ON_SPEED);
        double limitedSideways = Math.max(sideways.getAsDouble(), -1.0 * Constants.ModuleConstants.MAX_LOCKED_ON_SPEED);
        limitedSideways = Math.min(limitedSideways, Constants.ModuleConstants.MAX_LOCKED_ON_SPEED);

        drive.drive(limitedForward, limitedSideways, yawRate, true);

    }
 
}