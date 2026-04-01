package gcsrobotics.commands;

import java.util.function.Supplier;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.SleepCommand;

public class ShootCurrent implements Command {
    private final Supplier<ShootingPosition> positionSupplier;
    private SeriesCommand sequence;

    public ShootCurrent(Supplier<ShootingPosition> positionSupplier) {
        this.positionSupplier = positionSupplier;
    }

    @Override
    public void init() {
        OpModeBase robot = OpModeBase.INSTANCE;

        sequence = new SeriesCommand(
                // Step 1: stop intake and open gate simultaneously
                new InstantCommand(() -> {
                    robot.intakeMotor.setPower(0);
                    robot.gateServo.setPosition(Constants.Gate.OPEN_POSITION);
                }),
                // Step 2: wait 150ms for gate to open
                new SleepCommand(150),
                // Step 3: run intake to push artifact through
                new InstantCommand(() ->
                        robot.intakeMotor.setPower(Constants.Intake.FORWARD_POWER)),
                // Step 4: run intake for 2000ms
                new SleepCommand(2000),
                // Step 5: close gate and stop intake
                // flywheel keeps spinning at current velocity
                new InstantCommand(() -> {
                    robot.gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
                    robot.intakeMotor.setPower(0);
                })
        );
        sequence.init();
    }

    @Override
    public void loop() {
        sequence.loop();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }
}