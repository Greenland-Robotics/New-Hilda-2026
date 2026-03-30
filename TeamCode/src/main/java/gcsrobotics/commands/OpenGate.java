package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;
import gcsrobotics.pedroPathing.Constants;

public class OpenGate implements Command {
    private final OpModeBase robot = OpModeBase.INSTANCE;

    @Override
    public void init() {
        robot.gateServo.setPosition(Constants.Gate.OPEN_POSITION);
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return true;
    }
}