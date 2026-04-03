package gcsrobotics.commands;

import com.qualcomm.robotcore.hardware.DigitalChannel;

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
        boolean intake   = !OpModeBase.INSTANCE.intakeSensor.getState();
        boolean transfer = !OpModeBase.INSTANCE.transferSensor.getState();
        boolean shoot    = !OpModeBase.INSTANCE.shootSensor.getState();

        return intake && transfer && shoot;
    }
}