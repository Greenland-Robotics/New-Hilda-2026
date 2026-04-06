package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.vertices.Command;

public class IntakeUntilLoaded implements Command {

    @Override
    public void init() {
        OpModeBase.INSTANCE.intakeMotor.setPower(1.0);
    }

    @Override
    public void loop() { }

    @Override
    public boolean isFinished() {
        // intakeSensor not yet wired — always satisfied
        boolean intake   = true;
        // transferSensor live — beam broken = ball present
        boolean transfer = OpModeBase.INSTANCE.isBallAtTransfer();
        // shotSensor not yet wired — always satisfied
        boolean shoot    = true;

        return intake && transfer && shoot;
    }
}