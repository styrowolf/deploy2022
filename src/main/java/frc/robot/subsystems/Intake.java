// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  //TODO: check to see if Victor SPX motorcontrollers can be used as the master controller.
  public final WPI_VictorSPX intakeMaster = new WPI_VictorSPX(RobotMap.INTAKE_MASTER);
  public final WPI_VictorSPX intakeSlave = new WPI_VictorSPX(RobotMap.INTAKE_SLAVE);

  public Intake() {
    intakeSlave.follow(intakeMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    // -1.0 <= speed <= 1.0
    intakeMaster.set(ControlMode.PercentOutput, speed);
  }

  public double getMotorSpeed() {
    return intakeMaster.getMotorOutputPercent();
  }
}
