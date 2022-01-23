// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  //TODO: set deviceNumbers
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(0);
  private final WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(0);
  private final WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(0);

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(0);
  private final WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(0);
  private final WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(0);
  
  /* with MotorControllerGroup
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave1, rightSlave2);
  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave1, leftSlave2);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  */

  /* with master-slave */
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  //TODO: might need fix, check final/static
  public static DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

  //TODO: set constants
  PIDController turnController = new PIDController(0.0, 0.0, 0.0);
  AHRS navx = new AHRS(Port.kOnboardCS0 /* random thingy, change */);

  public Drive() {
    /* with master-slave */
    rightSlave1.follow(rightMaster);
    rightSlave2.follow(rightMaster);
    leftSlave1.follow(leftMaster);
    leftSlave2.follow(leftMaster);

    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
      0 /* PID_LOOP_ID */, 0 /* timeoutMs */);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 0 /* timeoutMs */);
    
    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
      0 /* PID_LOOP_ID */, 0 /* timeoutMs */);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 0 /* timeoutMs */);

    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(0.0 /* DriveConst.AngleTolerance */);
    
    odometry = new DifferentialDriveOdometry(new Rotation2d(navx.getAngle() / 180 * Math.PI));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
