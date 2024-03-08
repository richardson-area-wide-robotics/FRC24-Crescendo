package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveSubsystem;

public class BackUp extends SequentialCommandGroup {

    public BackUp(DriveSubsystem drive) {
        super(
            Commands.run(()-> drive.drive(-1,0, 0, false), drive).withTimeout(1.0)
        );
    }
    
}
