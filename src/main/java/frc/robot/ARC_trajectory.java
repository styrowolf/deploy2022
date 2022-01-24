package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.constants.DriveConst;
import frc.robot.subsystems.Drive;

public class ARC_trajectory {
    public Trajectory[] rpos1 = new Trajectory[2];
    public Trajectory[] rpos2 = new Trajectory[2];
    public Trajectory[] lpos1 = new Trajectory[4];
    public Trajectory[] lpos2 = new Trajectory[4];
    
    private Drive m_drive;
    
    public ARC_trajectory(Drive drive){
        m_drive=drive;
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConst.ksVolts, DriveConst.kvVoltSecondsPerMeter,
                                                DriveConst.kaVoltSecondsSquaredPerMeter),
                                DriveConst.kDriveKinematics, 8); 
        TrajectoryConfig configReversed = new TrajectoryConfig(DriveConst.kMaxSpeedMetersPerSecond,
                                DriveConst.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConst.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);

        configReversed.setReversed(true);

        TrajectoryConfig configForward = new TrajectoryConfig(DriveConst.kMaxSpeedMetersPerSecond,
                                DriveConst.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConst.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);
        double divisor=1.0;

        rpos1[0] = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(13 / divisor, -5.8 / divisor, new Rotation2d(0)),
                        new Pose2d(11.5 / divisor, -7.0 / divisor, new Rotation2d(0)),
                        new Pose2d(8 / divisor, -7.0 / divisor, new Rotation2d(0))),
            configReversed); 
        
        
        rpos2[0] = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(13 / divisor, -5.8 / divisor, new Rotation2d(0)),
                        new Pose2d(11.5 / divisor, -7.0 / divisor, new Rotation2d(0)),
                        new Pose2d(8 / divisor, -7.0 / divisor, new Rotation2d(0))),
            configReversed);   //TODO: Check values                     


        lpos1[0] = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(13 / divisor, -5.8 / divisor, new Rotation2d(0)),
                        new Pose2d(11.5 / divisor, -7.0 / divisor, new Rotation2d(0)),
                        new Pose2d(8 / divisor, -7.0 / divisor, new Rotation2d(0))),
            configReversed); 
        
        
        lpos2[0] = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(13 / divisor, -5.8 / divisor, new Rotation2d(0)),
                        new Pose2d(11.5 / divisor, -7.0 / divisor, new Rotation2d(0)),
                        new Pose2d(8 / divisor, -7.0 / divisor, new Rotation2d(0))),
            configReversed);   //TODO: Check values            



    } 


    public RamseteCommand getRamsete(Trajectory traj){
        return new RamseteCommand(traj, m_drive::getPose , new RamseteController(DriveConst.kRamseteB, DriveConst.kRamseteZeta), new SimpleMotorFeedforward(DriveConst.ksVolts, DriveConst.kvVoltSecondsPerMeter, DriveConst.kaVoltSecondsSquaredPerMeter), DriveConst.kDriveKinematics, m_drive::getWheelSpeeds, new PIDController(DriveConst.kPDriveVel, 0, 0), new PIDController(DriveConst.kPDriveVel, 0, 0), m_drive::tankDriveVolts, m_drive);
    }

}
