package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.constants.ElevatorConst;

public class Elevator extends SubsystemBase {
    
    public final static WPI_TalonSRX elevatorMaster = new WPI_TalonSRX(RobotMap.ELEVATOR_MASTER);
    public final static WPI_VictorSPX elevatorSlave = new WPI_VictorSPX(RobotMap.ELEVATOR_SLAVE);

    private static double targetHeight;

    public Elevator(double height, int kPIDLoopID, double kP, double kI, double kD, double kF) {

        elevatorSlave.follow(elevatorMaster);

        elevatorMaster.configForwardSoftLimitThreshold(ElevatorConst.fwdTreshold);
        elevatorMaster.configReverseSoftLimitThreshold(ElevatorConst.fwdTreshold);

        elevatorMaster.configMotionCruiseVelocity(ElevatorConst.cruiseVelocity);
        elevatorMaster.configMotionAcceleration(ElevatorConst.acceleration);

        elevatorMaster.configNeutralDeadband(ElevatorConst.neutralDeadband, ElevatorConst.timeout);

        elevatorMaster.configNominalOutputForward(ElevatorConst.fwdNominalOutput);
        elevatorMaster.configNominalOutputReverse(ElevatorConst.revNominalOutput);

        elevatorMaster.configPeakOutputForward(ElevatorConst.fwdPeakOutput);
        elevatorMaster.configPeakOutputReverse(ElevatorConst.revPeakOutput);

        elevatorMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, ElevatorConst.kPIDLoopID, ElevatorConst.timeout);

        elevatorMaster.config_kP(kPIDLoopID, kP);
        elevatorMaster.config_kI(kPIDLoopID, kI);
        elevatorMaster.config_kD(kPIDLoopID, kD);
        elevatorMaster.config_kF(kPIDLoopID, kF);

        elevatorMaster.setNeutralMode(NeutralMode.Brake);

        elevatorMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, ElevatorConst.timeout);

        targetHeight = height;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground angle: ", RobotContainer.navX.getAngle());
    }

    //Getter Methods
    public static double getTargetHeight() {
        return targetHeight;
    }

    public static double getMotorSpeed() {
        return elevatorMaster.getMotorOutputPercent();
    }

    //Setter Methods
    public static void setElevatorSpeed(double speed) {
        elevatorMaster.set(ControlMode.PercentOutput, speed);
    }

    //Other Methods
    public static void getToHeight(double height) {

    }
    
    public static void extendElevator() {
        elevatorMaster.set(ControlMode.PercentOutput, 0.8);
    }

    public static void retractElevator() {
      elevatorMaster.set(ControlMode.PercentOutput, -0.8);
  }
}

