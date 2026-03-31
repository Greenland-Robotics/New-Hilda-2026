package gcsrobotics.commands;

import java.util.function.Supplier;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.AwaitCommand;
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
        // Read currentPosition at the moment the button is pressed
        ShootingPosition position = positionSupplier.get();
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
                // Step 3: open gate
                new InstantCommand(() ->
                        robot.gateServo.setPosition(Constants.Gate.OPEN_POSITION)),
                // Step 4: brief pause for gate to open
                new SleepCommand(150),
                // Step 5: start intake at full power
                new InstantCommand(() ->
                        robot.intakeMotor.setPower(Constants.Intake.FORWARD_POWER)),
                // Step 6: wait for artifact to pass through
                new SleepCommand(Constants.Flywheel.SHOOT_DURATION_MS),
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