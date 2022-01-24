package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drive;
import frc.robot.ARCTrajectory;


public class rpos1 extends SequentialCommandGroup{

    public rpos1(ARCTrajectory a_traj, Shooter shooter, Intake intake, Drive drive){ 
        super(new SetShooterRPMPF(2900, shooter, true),
        new SetShooterRPMPF(2900, shooter, false).withTimeout(1.5).raceWith(new RunHopper("sync", hopper)),
        a_traj.getRamsete(a_traj.rpos1[0])
            .raceWith(new RunIntake(0.7, intake)
                .alongWith(new RunHopper("sync", hopper).alongWith(new RunShooter(-0.3, shooter)))),
        a_traj.getRamsete(a_traj.rpos1[1]).andThen(() -> drive.tankDriveVolts(0, 0)),
        new RunHopper("", hopper).withTimeout(0.2).alongWith(new VisionTurnCG(shooter, drive, led)),
        new SetShooterRPMPF(2900, shooter, false).withTimeout(1.5)
            .raceWith(new RunHopper("sync", hopper).alongWith(new RunIntake(0.7, intake))));


    }    
}
