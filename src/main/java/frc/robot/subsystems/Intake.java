// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  VictorSP intakeMaster = new VictorSP(RobotMap.INTAKE_MASTER);
  VictorSP intakeSlave = new VictorSP(RobotMap.INTAKE_SLAVE);
  MotorControllerGroup motors = new MotorControllerGroup(intakeMaster, intakeSlave);

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    // -1.0 <= speed <= 1.0
    motors.set(speed);
  }

  public double getMotorSpeed() {
    return motors.get();
  }
}
