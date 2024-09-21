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

public class FrontFourRightToLeftAuto extends SequentialCommandGroup {
    public FrontFourRightToLeftAuto(DriveSubsystem drive, Intake intake, Shooter shooter, Pivot pivot, Feeder feeder) {
        super(
        Commands.runOnce(()-> drive.zeroHeading(), drive)
        .andThen(Commands.runOnce(()-> drive.resetEncoders(), drive))
        .andThen(new WaitCommand(0.2))
        .andThen(pivot.pivotToSpeaker().withTimeout(0.5))
        .alongWith(Commands.runOnce(() -> shooter.setStateSpeaker(ShooterState.SPEAKER)))
        .andThen(new WaitCommand(0.2))
        .andThen(feeder.shootNote().withTimeout(0.5))

        .andThen(Commands.run(()-> drive.drive(-0.15, 0, 0, false), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).withTimeout(0.50))//.until(()->feeder.getIndicator() == true))
        .andThen(pivot.pivotToRange().withTimeout(0.5))
        .andThen(new WaitCommand(0.5))
        .andThen(feeder.shootNote().withTimeout(0.5))

        .andThen(Commands.run(()-> drive.drive(0, 0.1435, -0.14, true), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).withTimeout(2.0))
        .andThen(Commands.run(()-> drive.drive(0, -0.1435, 0.14, true), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).withTimeout(2.00))
        .andThen(feeder.shootNote().withTimeout(0.5))

        .andThen(Commands.run(()-> drive.drive(0, -0.1435, 0.08, true), drive)//0.10 rotation
        .alongWith(feeder.feedNote().alongWith(intake.intake())).withTimeout(2.00))
        .andThen(Commands.run(()-> drive.drive(0.17, 0, -0.1475, true), drive).withTimeout(0.6))
        .andThen(feeder.shootNote().withTimeout(0.5))

        .andThen(Commands.run(()-> drive.drive(-0.725, -0.4575, 0, true), drive).withTimeout(1.75))//-0.35, -0.2
        .andThen(Commands.run(()-> drive.drive(-0.275, 0, 0.01, true), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).withTimeout(2.00))
        );
    }
}
