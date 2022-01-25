package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;


public abstract class Controller {

    public static PSController getPS4Controller(int port) {
        return new PSController(port);
    }

    public static XController getXboxController(int port) {
        return new XController(port);
    }

    public static JoystickController getJoystickController(int port) {
        return new JoystickController(port);
    }


    //Intake
    public boolean getIntakeForward() {
        return false;
    }
    public boolean getIntakeReverse() {
        return false;
    }

    public double getDriveSpeed() {
        return 0;
    }
    public double getDriveRotation() {
        return 0;
    }
}

class PSController extends Controller {

    private PS4Controller controller;

    public PSController(int port) {
        controller = new PS4Controller(port);
    }

    @Override
    public double getDriveSpeed() {
        return controller.getLeftY();
    }
    
    @Override
    public double getDriveRotation() {
        return controller.getRightX();
    }
}

class XController extends Controller {

    private XboxController controller;

    public XController(int port) {
        controller = new XboxController(port);
    }

    @Override
    public double getDriveSpeed() {
        return controller.getLeftY();
    }
    
    @Override
    public double getDriveRotation() {
        return controller.getRightX();
    }
}

class JoystickController extends Controller {

    private Joystick controller;

    public JoystickController(int port) {
        controller = new Joystick(port);
    }

    @Override
    public double getDriveSpeed() {
        return controller.getY();
    }
    
    @Override
    public double getDriveRotation() {
        return controller.getX();
    }
}