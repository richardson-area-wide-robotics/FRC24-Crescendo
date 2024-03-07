package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ShootBackUp extends SequentialCommandGroup {

    public ShootBackUp(DriveSubsystem drive, Intake intake, Shooter shooter, Pivot pivot, Feeder feeder) {
      super(
        pivot.pivotToSpeaker().withTimeout(2.5)
        .alongWith(Commands.runOnce(() -> shooter.toggleState(ShooterState.SPEAKER)))
        .andThen(new WaitCommand(0.9))
        .andThen(feeder.shootNote().withTimeout(1.0))
        .andThen(Commands.runOnce(()-> shooter.toggleState(ShooterState.IDLE)))
        .andThen(Commands.run(()-> drive.drive(-1,0, 0, false), drive)
        .alongWith(intake.intake()).withTimeout(1.0)));
    } 
}
