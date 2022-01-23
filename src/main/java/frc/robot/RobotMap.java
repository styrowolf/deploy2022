package frc.robot;
import edu.wpi.first.wpilibj.SPI;

public class RobotMap {
    
    //
    // Drive
    //
    public static final SPI.Port navX = SPI.Port.kMXP;
    
    public static final int DRIVE_RIGHT_MASTER = 0;
    public static final int DRIVE_RIGHT_SLAVE_1 = 0;
    public static final int DRIVE_RIGHT_SLAVE_2 = 0;

    public static final int DRIVE_LEFT_MASTER = 0;
    public static final int DRIVE_LEFT_SLAVE_1 = 0;
    public static final int DRIVE_LEFT_SLAVE_2 = 0;
    //
    // Intake
    //
    public static final int INTAKE_MASTER = 0;
    public static final int INTAKE_SLAVE = 0;

    //
    // Elevator
    //
    public static final int ELEVATOR_MASTER_RIGHT = 0;
    public static final int ELEVATOR_MASTER_LEFT = 0;
    public static final int ELEVATOR_SLAVE_RIGHT = 0;
    public static final int ELEVATOR_SLAVE_LEFT = 0;

}
