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

public class TwoShootBasicAuto extends SequentialCommandGroup {
    
    public TwoShootBasicAuto(DriveSubsystem drive, Intake intake, Shooter shooter, Pivot pivot, Feeder feeder) {
        super(
        pivot.pivotToSpeaker().withTimeout(2.5)
        .alongWith(Commands.runOnce(() -> shooter.toggleState(ShooterState.SPEAKER)))
        .andThen(new WaitCommand(0.9))
        .andThen(feeder.shootNote().withTimeout(1.0))
        .andThen(Commands.run(()-> drive.drive(-0.5,0, 0, false), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).until(()->feeder.getIndicator() == true).withTimeout(1.5))
        .andThen(Commands.run(()-> drive.drive(0.35,0, 0, false), drive).withTimeout(3.0))
        .andThen(feeder.shootNote()));
    }
}
