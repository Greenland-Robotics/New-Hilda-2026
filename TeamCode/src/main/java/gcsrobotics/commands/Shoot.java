package gcsrobotics.commands;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.AwaitCommand;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.SleepCommand;

public class Shoot implements Command {
    private final ShootingPosition position;
    private SeriesCommand sequence;

    public Shoot(ShootingPosition position) {
        this.position = position;
    }

    @Override
    public void init() {
        OpModeBase robot = OpModeBase.INSTANCE;

        sequence = new SeriesCommand(
                // Step 1: spin up flywheels to target velocity and set hood angle
                new InstantCommand(() -> {
                    robot.setFlywheelVelocity(position.targetVelocity);
                    robot.hoodServo.setPosition(position.hoodPosition);
                }),
                // Step 2: wait for flywheels to reach target velocity
                new AwaitCommand(
                        () -> robot.getFlywheelVelocity() >=
                                position.targetVelocity * Constants.Flywheel.RPM_THRESHOLD,
                        Constants.Flywheel.FLYWHEEL_TIMEOUT_MS
                ),
                // Step 3: stop intake and open gate simultaneously
                new InstantCommand(() -> {
                    robot.intakeMotor.setPower(0);
                    robot.gateServo.setPosition(Constants.Gate.OPEN_POSITION);
                }),
                // Step 4: wait 150ms for gate to open
                new SleepCommand(150),
                // Step 5: run intake to push artifact through
                new InstantCommand(() ->
                        robot.intakeMotor.setPower(Constants.Intake.FORWARD_POWER)),
                // Step 6: run intake for 2000ms
                new SleepCommand(2000),
                // Step 7: close gate and stop intake
                // flywheel keeps spinning at current velocity until next input
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