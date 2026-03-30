package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

public class StopIntake implements Command {
    private final OpModeBase robot = OpModeBase.INSTANCE;

    @Override
    public void init() {
        robot.intakeMotor.setPower(0);
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return true;
    }
}