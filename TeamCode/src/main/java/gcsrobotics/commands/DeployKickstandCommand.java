package gcsrobotics.commands;

import gcsrobotics.vertices.Command;
import gcsrobotics.pedroPathing.Constants;

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
    public void loop() {
        // motor runs continuously while button held — nothing to update
    }

    @Override
    public boolean isFinished() {
        // never self-terminates — ButtonAction cancels this when button is released
        return false;
    }
}