package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.constants.DElevatorConst;

public class DoubleElevator extends SubsystemBase{
    
    public static final WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.ELEVATOR_MASTER_RIGHT);
    public static final WPI_VictorSPX rightSlave = new WPI_VictorSPX(RobotMap.ELEVATOR_SLAVE_RIGHT);

    public static final WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.ELEVATOR_MASTER_LEFT);
    public static final WPI_VictorSPX leftSlave = new WPI_VictorSPX(RobotMap.ELEVATOR_SLAVE_LEFT);

    private static double targetHeight_right;
    private static double targetHeight_left;

    public final PIDController anglePID = new PIDController(DElevatorConst.anglekP, DElevatorConst.anglekI, DElevatorConst.anglekD);

    public DoubleElevator() {

        rightSlave.follow(rightMaster);
        leftSlave.follow(leftMaster);

        leftMaster.setInverted(true);
        
        rightMaster.configForwardSoftLimitThreshold(DElevatorConst.fwdTreshold_right);
        leftMaster.configForwardSoftLimitThreshold(DElevatorConst.fwdTreshold_left);
        rightMaster.configReverseSoftLimitThreshold(DElevatorConst.revTreshold_left);
        leftMaster.configReverseSoftLimitThreshold(DElevatorConst.revTreshold_right);

        rightMaster.configMotionCruiseVelocity(DElevatorConst.velocity_right, DElevatorConst.timeout_right);
        leftMaster.configMotionCruiseVelocity(DElevatorConst.velocity_left, DElevatorConst.timeout_left);

        rightMaster.configMotionAcceleration(DElevatorConst.acceleration_right, DElevatorConst.timeout_right);
        leftMaster.configMotionAcceleration(DElevatorConst.accelertation_left, DElevatorConst.timeout_left);

        rightMaster.configNeutralDeadband(DElevatorConst.neutralDeadband_right, DElevatorConst.timeout_right);
        leftMaster.configNeutralDeadband(DElevatorConst.neutralDeadband_left, DElevatorConst.timeout_left);

        rightMaster.configNominalOutputForward(DElevatorConst.fwdNominalOutput_right, DElevatorConst.timeout_right);
        leftMaster.configNominalOutputForward(DElevatorConst.fwdNominalOutput_left, DElevatorConst.timeout_left);
        rightMaster.configNominalOutputReverse(DElevatorConst.revNominalOutput_right, DElevatorConst.timeout_right);
        leftMaster.configNominalOutputReverse(DElevatorConst.revNominalOutput_left, DElevatorConst.timeout_left);

        rightMaster.configPeakOutputForward(DElevatorConst.fwdPeakOutput_right, DElevatorConst.timeout_right);
        leftMaster.configPeakOutputForward(DElevatorConst.fwdPeakOutput_left, DElevatorConst.timeout_left);
        rightMaster.configPeakOutputForward(DElevatorConst.revPeakOutput_right, DElevatorConst.timeout_right);
        leftMaster.configPeakOutputForward(DElevatorConst.revPeakOutput_left, DElevatorConst.timeout_left);

        rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, DElevatorConst.kPIDLoopID_right, DElevatorConst.timeout_right);
        leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, DElevatorConst.kPIDLoopID_left, DElevatorConst.timeout_left);

        rightMaster.config_kP(DElevatorConst.kPIDLoopID_right, DElevatorConst.kP_right);
        rightMaster.config_kI(DElevatorConst.kPIDLoopID_right, DElevatorConst.kI_right);
        rightMaster.config_kD(DElevatorConst.kPIDLoopID_right, DElevatorConst.kD_right);
        rightMaster.config_kF(DElevatorConst.kPIDLoopID_right, DElevatorConst.kF_right);

        leftMaster.config_kP(DElevatorConst.kPIDLoopID_left, DElevatorConst.kP_left);
        leftMaster.config_kI(DElevatorConst.kPIDLoopID_left, DElevatorConst.kI_left);
        leftMaster.config_kD(DElevatorConst.kPIDLoopID_left, DElevatorConst.kD_left);
        leftMaster.config_kF(DElevatorConst.kPIDLoopID_left, DElevatorConst.kF_left);

        rightMaster.setNeutralMode(NeutralMode.Brake);
        leftMaster.setNeutralMode(NeutralMode.Brake);

        rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, DElevatorConst.timeout_right);
        leftMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, DElevatorConst.timeout_left);

        targetHeight_right = 0.0;
        targetHeight_left = 0.0;

        anglePID.enableContinuousInput(-180, 180);
        anglePID.setTolerance(DElevatorConst.angleTolerance);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground angle: ", RobotContainer.navX.getAngle());
    }

    //Getter Methods
    public static double getTargetHeight_Left() {
        return targetHeight_left;
    }

    public static double getTargetHeight_Right() {
        return targetHeight_right;
    }

    public static double getMotorSpeed_Left() {
        return leftMaster.getMotorOutputPercent();
    }

    public static double getMotorSpeed_Right() {
        return rightMaster.getMotorOutputPercent();
    }

    //Setter Methods
    public static void setElevatorSpeed(double speed) {
        rightMaster.set(ControlMode.PercentOutput, speed);
        leftMaster.set(ControlMode.PercentOutput, speed);
    }

    public static void setElevatorSpeed_Right(double speed) {
        rightMaster.set(ControlMode.PercentOutput, speed);
    }

    public static void setElevatorSpeed_Left(double speed) {
        leftMaster.set(ControlMode.PercentOutput, speed);
    }

    //Other Methods
    public static void getToHeight() {

    }

    public static void extendElevator_Right() {
        rightMaster.set(ControlMode.PercentOutput, 0.8);
    }

    public static void extendElevator_Left() {
        leftMaster.set(ControlMode.PercentOutput, 0.8);
    }

    public static void retractElevator_Right() {
        rightMaster.set(ControlMode.PercentOutput, -0.8);
    }

    public static void retractElevator_Left() {
        leftMaster.set(ControlMode.PercentOutput, -0.8);
    }
}
