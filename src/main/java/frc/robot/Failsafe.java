package frc.robot;

import java.util.function.Supplier;

public class Failsafe {
    private boolean state = false;
    private Supplier<Boolean> activated;
    
    public Failsafe(Supplier<Boolean> activateFailsafe) {
        activated = activateFailsafe;
    } 

    public boolean check() {
        if (!state) {
            state = activated.get();
        }
        return state;
    }

    public void checkAndExit() {
        if (check()) {
            System.exit(0);
        }
    }
}
