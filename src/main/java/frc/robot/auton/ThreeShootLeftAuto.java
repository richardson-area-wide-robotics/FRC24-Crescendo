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

public class ThreeShootLeftAuto extends SequentialCommandGroup {
    public ThreeShootLeftAuto(DriveSubsystem drive, Intake intake, Shooter shooter, Pivot pivot, Feeder feeder) {
        super(
        Commands.runOnce(()-> drive.zeroHeading(), drive)
        .andThen(Commands.runOnce(()-> drive.resetEncoders(), drive))
        .andThen(new WaitCommand(0.2))
        .andThen(pivot.pivotToSpeaker().withTimeout(0.5))
        .alongWith(Commands.runOnce(() -> shooter.setStateSpeaker(ShooterState.SPEAKER)))
        .andThen(new WaitCommand(0.2))
        .andThen(feeder.shootNote().withTimeout(0.5))

        .andThen(Commands.run(()-> drive.drive(-0.15, 0, 0, false), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).until(()->feeder.getIndicator() == true))
        .andThen(Commands.run(()-> drive.drive(0.25, 0, 0, false), drive).withTimeout(0.825))//0.125, 2.0
        .andThen(feeder.shootNote().withTimeout(0.5))

        .andThen(Commands.run(()-> drive.drive(-0.087, -0.14, 0, false), drive).withTimeout(2.05))
        .andThen(Commands.run(()-> drive.drive(-0.15, 0, 0, false), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).withTimeout(0.75)//.until(()->feeder.getIndicator() == true))

        .andThen(Commands.run(()-> drive.drive(0, 0, 0.2, false), drive)
        .alongWith(feeder.feedNote().alongWith(intake.intake())).withTimeout(0.65))
        .andThen(Commands.run(()-> drive.drive(0.2, -0.1, 0, false), drive).withTimeout(2.0)))
        .andThen(feeder.shootNote().withTimeout(0.5))
        );
    }
}
