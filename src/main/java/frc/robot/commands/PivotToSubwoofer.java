// package frc.robot.commands;

// import static edu.wpi.first.units.Units.*;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.Constants.ShooterConstants;

// public class PivotToSubwoofer extends Command {
//     private Shooter m_shooter;

//     public PivotToSubwoofer(Shooter shooter) {
//         m_shooter = shooter;
//         addRequirements(m_shooter);
//     }

//     @Override
//     public void execute() {
//         // m_shooter.pivotTo(ShooterConstants.PIVOT_PRESET_SUBWOOFER);
//     }

//     @Override
//     public boolean isFinished() {
//         return ShooterConstants.PIVOT_PRESET_SUBWOOFER - m_shooter.getCurrentPivotAngle().in(Radians) < ShooterConstants.kPivotToleranceAngle.in(Radians);
//     }
// }
