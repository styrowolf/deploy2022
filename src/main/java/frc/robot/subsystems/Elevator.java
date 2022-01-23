package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  
  public WPI_TalonSRX elevatorMasterRight = new WPI_TalonSRX(RobotMap.ELEVATOR_MASTER_RIGHT);
  public WPI_TalonSRX elevatorMasterLeft = new WPI_TalonSRX(RobotMap.ELEVATOR_MASTER_LEFT);
  
  public WPI_VictorSPX elevatorSlaveRight = new WPI_VictorSPX(RobotMap.ELEVATOR_SLAVE_RIGHT);
  public WPI_VictorSPX elevatorSlaveLeft = new WPI_VictorSPX(RobotMap.ELEVATOR_SLAVE_LEFT);

  public enum ElevatorStateMachine {
    DISABLED,
    MANUAL,
    PID
  }

  public Elevator() {
    elevatorSlaveRight.follow(elevatorMasterRight);
    elevatorSlaveLeft.follow(elevatorMasterLeft);

    elevatorMasterLeft.setInverted(true);
  }

  @Override
  public void periodic() {
  
  }

  public void setElevatorSpeedRight(double speed) {
    elevatorMasterRight.set(ControlMode.PercentOutput, speed);
  }

  public void setElevatorSpeedLeft(double speed) {
    elevatorMasterLeft.set(ControlMode.PercentOutput, speed);
  }

  public double getMotorSpeedRight() {
    return elevatorMasterRight.getMotorOutputPercent();
  }

  public double getMotorSpeedLeft() {
    return elevatorMasterLeft.getMotorOutputPercent();
  }

}
