package gcsrobotics.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import gcsrobotics.commands.CloseGate;
import gcsrobotics.commands.DeployKickstandCommand;
import gcsrobotics.commands.KickstandSubsystem;
import gcsrobotics.commands.OpenGate;
import gcsrobotics.commands.RetractKickstandCommand;
import gcsrobotics.commands.SetHoodAngle;
import gcsrobotics.commands.Shoot;
import gcsrobotics.commands.ShootingPosition;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.control.TeleOpBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.ButtonAction;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;

// ============================================================
//  Hilda TeleOp — NH Premier
// ============================================================
//
//  GAMEPAD 1 — DRIVER
//  ----------------------------------------------------------
//  Left stick          Robot-centric drive (strafe / forward)
//  Right stick X       Rotation
//  A                   Snap-to-shoot CLOSE  (blue default)
//  B                   Snap-to-shoot MEDIUM (blue default)
//  X                   Snap-to-shoot TOP    (blue default)
//  Y                   Snap-to-shoot FAR    (blue default)
//  D-pad Up            Deploy kickstand
//  D-pad Down          Retract kickstand
//
//  GAMEPAD 2 — OPERATOR
//  ----------------------------------------------------------
//  Left stick Y        Intake (forward = intake, back = reverse)
//  Left bumper         Shoot sequence at current position
//  Right bumper        Shoot sequence at current position
//  Left trigger        Open gate manually
//  Right trigger       Close gate manually
//
//  ALLIANCE SELECTION
//  ----------------------------------------------------------
//  Press Start + A (GP1) before init to select BLUE (default)
//  Press Start + B (GP1) before init to select RED
//  Telemetry shows current alliance at all times.
//
// ============================================================

@TeleOp(name = "Gearmo TeleOp — Red", group = "Gearmo")
public class GearmoTeleopRed extends TeleOpBase {

    // ---- Alliance selection ----
    private boolean isBlue = true;

    // ---- Shooting position tracker ----
    private ShootingPosition currentPosition = ShootingPosition.CLOSE_BLUE;

    // ---- Kickstand ----
    private KickstandSubsystem kickstand;

    // ---- Command runner ----
    private CommandRunner commandRunner;

    // ---- Button actions ----
    private ButtonAction snapClose;
    private ButtonAction snapMedium;
    private ButtonAction snapTop;
    private ButtonAction snapFar;
    private ButtonAction shootLeft;
    private ButtonAction shootRight;
    private ButtonAction deployKickstand;
    private ButtonAction retractKickstand;

    @Override
    protected void initialize() {
        OpModeBase robot = OpModeBase.INSTANCE;

        kickstand = new KickstandSubsystem(hardwareMap.get(CRServo.class, "kickstand"));
        commandRunner = new CommandRunner();

        buildActions();

        telemetry.addData("Hilda TeleOp", "Initialized");
        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.update();
    }

    private void buildActions() {

        // GP1 ABXY: set hood angle + flywheel velocity + track current position
        snapClose = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = isBlue ? ShootingPosition.CLOSE_BLUE : ShootingPosition.CLOSE_RED;
                            double radPerSec = currentPosition.targetRPM * 2 * Math.PI / 60.0;
                            OpModeBase.INSTANCE.setFlywheelVelocity(radPerSec);
                        }),
                        new SetHoodAngle(isBlue
                                ? ShootingPosition.CLOSE_BLUE : ShootingPosition.CLOSE_RED)
                ), commandRunner
        );

        snapMedium = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = isBlue ? ShootingPosition.MEDIUM_BLUE : ShootingPosition.MEDIUM_RED;
                            double radPerSec = currentPosition.targetRPM * 2 * Math.PI / 60.0;
                            OpModeBase.INSTANCE.setFlywheelVelocity(radPerSec);
                        }),
                        new SetHoodAngle(isBlue
                                ? ShootingPosition.MEDIUM_BLUE : ShootingPosition.MEDIUM_RED)
                ), commandRunner
        );

        snapTop = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = isBlue ? ShootingPosition.TOP_BLUE : ShootingPosition.TOP_RED;
                            double radPerSec = currentPosition.targetRPM * 2 * Math.PI / 60.0;
                            OpModeBase.INSTANCE.setFlywheelVelocity(radPerSec);
                        }),
                        new SetHoodAngle(isBlue
                                ? ShootingPosition.TOP_BLUE : ShootingPosition.TOP_RED)
                ), commandRunner
        );

        snapFar = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = isBlue ? ShootingPosition.FAR_BLUE : ShootingPosition.FAR_RED;
                            double radPerSec = currentPosition.targetRPM * 2 * Math.PI / 60.0;
                            OpModeBase.INSTANCE.setFlywheelVelocity(radPerSec);
                        }),
                        new SetHoodAngle(isBlue
                                ? ShootingPosition.FAR_BLUE : ShootingPosition.FAR_RED)
                ), commandRunner
        );

        // GP2 Bumpers: shoot at the last snapped position
        shootLeft = new ButtonAction(
                new Shoot(currentPosition),
                commandRunner
        );

        shootRight = new ButtonAction(
                new Shoot(currentPosition),
                commandRunner
        );

        // GP1 D-pad: kickstand
        deployKickstand = new ButtonAction(
                new DeployKickstandCommand(kickstand), commandRunner
        );

        retractKickstand = new ButtonAction(
                new RetractKickstandCommand(kickstand), commandRunner
        );
    }

    @Override
    protected void runLoop() {
        OpModeBase robot = OpModeBase.INSTANCE;

        // Alliance selection (before match — Start+A = blue, Start+B = red)
        if (gamepad1.start && gamepad1.a) {
            isBlue = true;
        } else if (gamepad1.start && gamepad1.b) {
            isBlue = false;
        }

        // GP1: snap to shooting position
        snapClose.update(gamepad1.a && !gamepad1.start);
        snapMedium.update(gamepad1.b && !gamepad1.start);
        snapTop.update(gamepad1.x);
        snapFar.update(gamepad1.y);

        // GP1: kickstand
        deployKickstand.update(gamepad1.dpad_up);
        retractKickstand.update(gamepad1.dpad_down);

        // GP2: intake — analog left stick (forward = in, back = unjam)
        double intakeInput = -gamepad2.left_stick_y;
        robot.intakeMotor.setPower(Math.abs(intakeInput) > 0.05 ? intakeInput : 0);

        // GP2: shoot sequence
        shootLeft.update(gamepad2.left_bumper);
        shootRight.update(gamepad2.right_bumper);

        // GP2: manual gate override
        if (gamepad2.left_trigger > 0.1) {
            robot.gateServo.setPosition(Constants.Gate.OPEN_POSITION);
        } else if (gamepad2.right_trigger > 0.1) {
            robot.gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
        }

        // Telemetry
        telemetry.addData("Alliance",         isBlue ? "BLUE" : "RED");
        telemetry.addData("Hood Position",    currentPosition);
        telemetry.addData("Flywheel Vel",     robot.getFlywheelVelocity());
        telemetry.addData("Intake Power",     robot.intakeMotor.getPower());
        telemetry.addData("X",                robot.follower.getPose().getX());
        telemetry.addData("Y",                robot.follower.getPose().getY());
        telemetry.addData("Heading (deg)",    Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.update();
    }
}