package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveConst {
  //
  //CTRE SRX Mag Encoder
  //
 
    public static final int PID_LOOP_ID = 0;
    
    public static final int TIMEOUT = 30;

    public static final int ENCODER_CPR = 2048;

  //
  //PID Constants
  //
  public static final double kP = 0;

  public static final double kI = 0;

  public static final double kD = 0;

  public static final double ANGLE_TOLERANCE = 0.5;

  //
  //Drivebase
  //
  public static final double TRACK_WIDTH_METERS = 0.69; //TODO: Design verification

  public static final DifferentialDriveKinematics DriveKinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

  public static final double WHEEL_DIA_METERS = 6 * 0.0254;

  public static final double MAX_OUTPUT = 1.0;

  //
  //Drivebase Characteristics
  //

  public static final double ksVolts = 0.22;  //TODO: Configure
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;

   
  public static final double MAX_DRIVE_OUTPUT = 1.0;
  //
  //Auto Constants
  //


  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static final double kPDriveVel = 8.5; //TODO: Configure


}
