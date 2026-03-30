package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

public class SetHoodAngle implements Command {
    private final ShootingPosition position;

    public SetHoodAngle(ShootingPosition position) {
        this.position = position;
    }

    @Override
    public void init() {
        OpModeBase.INSTANCE.hoodServo.setPosition(position.hoodPosition);
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return true;
    }
}