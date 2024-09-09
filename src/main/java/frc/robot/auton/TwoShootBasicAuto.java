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
        Commands.runOnce(()-> drive.zeroHeading(), drive).
        andThen(Commands.runOnce(()-> drive.resetEncoders(), drive)).
        andThen(new WaitCommand(0.5)).
        andThen(pivot.pivotToSpeaker().withTimeout(1.0))
        .alongWith(Commands.runOnce(() -> shooter.setStateSpeaker(ShooterState.SPEAKER)))
        .andThen(new WaitCommand(0.4))
        .andThen(feeder.shootNote().withTimeout(1.0))
        .andThen(Commands.run(()-> drive.drive(-0.15,0, 0, false), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).until(()->feeder.getIndicator() == true))
        .andThen(Commands.run(()-> drive.drive(0.125,0, 0, false), drive).withTimeout(2.0))
        .andThen(feeder.shootNote()));
    }
}
