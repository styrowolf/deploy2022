// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.constants.DriveConst;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends SubsystemBase {

  //TODO: set deviceNumbers
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(0);
  private final WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(0);
  private final WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(0);

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(0);
  private final WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(0);
  private final WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(0);
  
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave1, rightSlave2);
  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave1, leftSlave2);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

  public static DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

  PIDController turnController = new PIDController(0.0, 0.0, 0.0);
  
  AHRS navX = new AHRS(RobotMap.navX);

  public Drive() {

    leftMotors.setInverted(true);

    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
      0 /* PID_LOOP_ID */, 0 /* timeoutMs */);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 0 /* timeoutMs */);
    
    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 
      0 /* PID_LOOP_ID */, 0 /* timeoutMs */);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 0 /* timeoutMs */);

    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(DriveConst.ANGLE_TOLERANCE);
    
    odometry = new DifferentialDriveOdometry(new Rotation2d(navX.getAngle() / 180 * Math.PI));

    setMaxOutput(DriveConst.MAX_DRIVE_OUTPUT);

    resetEncoders();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Wheel distance;", toMeters(getEncoderPos(rightMaster)));
    SmartDashboard.putNumber("Left Wheel distance;", toMeters(getEncoderPos(leftMaster)));
    
    SmartDashboard.putNumber("Right Wheel velocity;", toMeters(getEncoderVel(rightMaster)));
    SmartDashboard.putNumber("Left Wheel velocity;", toMeters(getEncoderVel(leftMaster)));

    odometry.update(navX.getRotation2d(), getEncoderPos(leftMaster), getEncoderPos(rightMaster));
  }
  
  //
  //Movement Commands
  //

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    differentialDrive.feed();
  }

  public void PIDTurn(double rotSpeed, double targetAngle) {
    arcadeDrive(rotSpeed, MathUtil.clamp(turnController.calculate(getHeading(), targetAngle), -0.8, 0.8));
  }
  
  //
  //Resets
  //

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, new Rotation2d(0.0));
  }

  public void resetEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
  }

  public void zeroHeading() {
    navX.reset();
  }

  //
  //Getter Commands
  //

  public double getEncoderPos(TalonSRX encoder) {
    return encoder.getSelectedSensorPosition();
  }

  public double getEncoderVel(TalonSRX encoder) {
    return encoder.getSelectedSensorVelocity();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      toMetersPerSec(rightMaster.getSelectedSensorVelocity()), toMetersPerSec(leftMaster.getSelectedSensorVelocity()));
  }
  
  public double getAverageEncoderDistance() {
    return (toMeters(getEncoderPos(rightMaster)) + toMeters(getEncoderPos(rightMaster)) / 2.0);
  }

  public double getHeading() {
    return navX.getAngle();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  //
  //Conversion Methods
  //

  public double toMetersPerSec(double sensorVelocity) {
    return sensorVelocity; // TODO: Designdan çark oranlarına bakılacak
  }

  public double toSensorVel(double metersPerSec) {
    return metersPerSec;
  }

  public double toSensorUnits(double meters) {
    return meters;
  }

  public double toMeters(double sensorUnits) {
    return sensorUnits;
  }
}


