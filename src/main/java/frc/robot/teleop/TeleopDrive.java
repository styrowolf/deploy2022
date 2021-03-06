// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TeleopDrive extends CommandBase {
  /** Creates a new TeleopDrive. */
  public TeleopDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.drive.arcadeDrive(RobotContainer.controller.getDriveSpeed(), RobotContainer.controller.getDriveRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
