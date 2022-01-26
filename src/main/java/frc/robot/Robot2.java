/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot2 extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  
   /* Doğru motorları configleyin!!!
  private  leftMotor1 = new Spark(0);
  private Spark leftMotor2 = new Spark(1);
  private Spark rightMotor1 = new Spark(2);
  private Spark rightMotor2 = new Spark(3);
 */

    public final WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
    private final WPI_VictorSPX rightSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_SLAVE_1);
    private final WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_SLAVE_2);

    public final WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
    private final WPI_VictorSPX leftSlave1 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_SLAVE_1);
    private final WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_SLAVE_2);

    MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave1, rightSlave2);
    MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave1, leftSlave2);
  private Joystick joy1 = new Joystick(0);

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

  @Override
  public void robotInit() {
    encoder.reset();
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousInit() {
    
  }

  final double kP = 0.5;

  double setpoint = 0;

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value (tick -> feet)", encoder.get() * kDriveTick2Feet);
    SmartDashboard.putNumber("encoder value (left/tick -> feet)", leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("encoder value (right/tick -> feet)", rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);
  }

  @Override
  public void teleopInit() {
    encoder.reset();
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    // get joystick command
    if (joy1.getRawButton(1)) {
        setpoint = 10;
    } else if (joy1.getRawButton(2)) {
        setpoint = 0;
    }
  
      // get sensor position
      //double sensorPosition = encoder.get() * kDriveTick2Feet;


        double sensorPositionLeft = leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
        double sensorPositionRight = rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
  
      // calculations
      //double error = setpoint - sensorPosition;
      double error = setpoint - (sensorPositionLeft + sensorPositionRight) / 2;
  
      double outputSpeed = kP * error;
  
      // output to motors
      leftMotors.set(outputSpeed);
      rightMotors.set(-outputSpeed);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
