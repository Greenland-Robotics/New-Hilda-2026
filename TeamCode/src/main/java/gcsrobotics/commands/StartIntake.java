package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;
import gcsrobotics.pedroPathing.Constants;

public class StartIntake implements Command {
    private final OpModeBase robot = OpModeBase.INSTANCE;

    @Override
    public void init() {
        robot.intakeMotor.setPower(Constants.Intake.FORWARD_POWER);
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return true; // fire and forget - intake runs until StopIntake is called
    }
}