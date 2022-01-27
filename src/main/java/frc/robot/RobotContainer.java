package frc.robot;

import frc.robot.subsystems.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    
    private static final RobotContainer mRobotContainer = new RobotContainer();

    public static Controller controller = Controller.getPS4Controller(0);
    public static Drive drive = new Drive();
    //public static AHRS navX = new AHRS(RobotMap.navX);

    SendableChooser<Command> m_chooser = new SendableChooser<Command>();

    private RobotContainer() {

    }

    //Getters

    public Command getAutonomousCommand() {

        return m_chooser.getSelected();

    }

    public static RobotContainer getInstance() {

        return mRobotContainer;
        
    }

}
