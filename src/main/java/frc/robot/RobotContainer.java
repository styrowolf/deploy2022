package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    
    private static final RobotContainer mRobotContainer = new RobotContainer();

    public static Controller controller = new Controller();
    public static Intake intake = new Intake();

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
