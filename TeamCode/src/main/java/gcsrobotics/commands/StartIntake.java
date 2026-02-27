package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.Command;

/// ======== Example Command ========
public class StartIntake implements Command {
    OpModeBase robot = OpModeBase.INSTANCE;

    public void init() {}

    public void loop() {
        robot.intake.setVelocity(Constants.INTAKE_VELOCITY);
    }

    public boolean isFinished() {return true;}
}
