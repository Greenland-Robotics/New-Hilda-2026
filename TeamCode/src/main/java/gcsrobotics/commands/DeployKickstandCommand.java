package gcsrobotics.commands;

import gcsrobotics.vertices.Command;

public class DeployKickstandCommand implements Command {

    private final KickstandSubsystem kickstand;

    public DeployKickstandCommand(KickstandSubsystem kickstand) {
        this.kickstand = kickstand;
    }

    @Override
    public void init() {
        kickstand.deploy();
    }

    @Override
    public void loop() {}

    @Override
    public boolean isFinished() {
        return false;
    }
}