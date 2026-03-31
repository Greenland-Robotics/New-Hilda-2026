package gcsrobotics.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import gcsrobotics.commands.DeployKickstandCommand;
import gcsrobotics.commands.FollowPath;
import gcsrobotics.commands.GoToHumanPlayer;
import gcsrobotics.commands.KickstandSubsystem;
import gcsrobotics.commands.Park;
import gcsrobotics.commands.RetractKickstandCommand;
import gcsrobotics.commands.SetHoodAngle;
import gcsrobotics.commands.ShootCurrent;
import gcsrobotics.commands.ShootingPosition;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.control.TeleOpBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.ButtonAction;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;

@TeleOp(name = "Gearmo TeleOp — NH Premier", group = "Hilda")
public class GearmoTeleopRed extends TeleOpBase {

    // ============================================================
    //  GAMEPAD 1 — DRIVER
    //  Left stick Y/X      Forward / strafe
    //  Right stick X       Rotation
    //  Left bumper         Strafe left
    //  Right bumper        Strafe right
    //  A                   Path to CLOSE shooting position
    //  B                   Path to MEDIUM shooting position
    //  X                   PathToCenter
    //  Y                   PathToFarZone
    //  D-pad Up            Deploy kickstand
    //  D-pad Down          Retract kickstand
    //  D-pad Right         GoToHumanPlayer
    //  D-pad Left          Park
    //  Start + A           Set alliance → BLUE
    //  Start + B           Set alliance → RED
    //
    //  GAMEPAD 2 — OPERATOR
    //  Left stick Y        Intake (forward = in, pull back = unjam)
    //  A                   Hood → CLOSE
    //  B                   Hood → MEDIUM
    //  X                   Hood → TOP
    //  Y                   Hood → FAR
    //  Right bumper        Shoot at current position
    //  Left bumper         Stop flywheel
    //  Left trigger        Manual gate open
    //  Right trigger       Manual gate close
    // ============================================================

    // ---- Alliance ----
    private boolean isBlue = true;

    // ---- Current shooting position (updated by GP2 A/B/X/Y) ----
    private ShootingPosition currentPosition = ShootingPosition.CLOSE;

    // ---- Kickstand ----
    private KickstandSubsystem kickstand;

    // ---- GP1 button actions ----
    private ButtonAction snapClose;
    private ButtonAction snapMedium;
    private ButtonAction snapTop;
    private ButtonAction snapFar;
    private ButtonAction goToHumanPlayer;
    private ButtonAction pathToPark;
    private ButtonAction deployKickstand;
    private ButtonAction retractKickstand;

    // ---- GP2 button actions ----
    private ButtonAction hoodClose;
    private ButtonAction hoodMedium;
    private ButtonAction hoodTop;
    private ButtonAction hoodFar;
    private ButtonAction shoot;
    private ButtonAction stopFlywheel;
    private ButtonAction openGate;
    private ButtonAction closeGate;

    @Override
    protected void initialize() {
        kickstand = new KickstandSubsystem(
                hardwareMap.get(CRServo.class, Constants.Kickstand.MOTOR_NAME)
        );
        buildActions();

        telemetry.addData("Status",   "Initialized");
        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.update();
    }

    // Called once at init and again any time alliance changes
    private void buildActions() {

        // ---- GP1: Path to shooting position (path + heading, no shoot) ----
        snapClose = new ButtonAction(
                new FollowPath(ShootingPosition.CLOSE.poseX,
                        ShootingPosition.CLOSE.poseY,
                        ShootingPosition.CLOSE.poseHeading),
                commandRunner
        );
        snapMedium = new ButtonAction(
                new FollowPath(ShootingPosition.MEDIUM.poseX,
                        ShootingPosition.MEDIUM.poseY,
                        ShootingPosition.MEDIUM.poseHeading),
                commandRunner
        );

        // ---- GP1: X → PathToCenter, Y → PathToFarZone ----
        snapTop = new ButtonAction(
                new gcsrobotics.commands.PathToCenter(isBlue), commandRunner
        );
        snapFar = new ButtonAction(
                new gcsrobotics.commands.PathToFarZone(isBlue), commandRunner
        );

        // ---- GP1: D-pad ----
        goToHumanPlayer = new ButtonAction(
                new GoToHumanPlayer(isBlue), commandRunner
        );
        pathToPark = new ButtonAction(
                new Park(isBlue), commandRunner
        );

        // ---- GP1: Kickstand ----
        deployKickstand = new ButtonAction(
                new DeployKickstandCommand(kickstand), commandRunner
        );
        retractKickstand = new ButtonAction(
                new RetractKickstandCommand(kickstand), commandRunner
        );

        // ---- GP2: Hood position (updates currentPosition for shoot) ----
        hoodClose = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> currentPosition = ShootingPosition.CLOSE),
                        new SetHoodAngle(ShootingPosition.CLOSE)
                ), commandRunner
        );
        hoodMedium = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> currentPosition = ShootingPosition.MEDIUM),
                        new SetHoodAngle(ShootingPosition.MEDIUM)
                ), commandRunner
        );
        hoodTop = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> currentPosition = ShootingPosition.TOP),
                        new SetHoodAngle(ShootingPosition.TOP)
                ), commandRunner
        );
        hoodFar = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> currentPosition = ShootingPosition.FAR),
                        new SetHoodAngle(ShootingPosition.FAR)
                ), commandRunner
        );

        // ---- GP2: Shoot at current position ----
        shoot = new ButtonAction(
                new ShootCurrent(() -> currentPosition), commandRunner
        );

        // ---- GP2: Stop flywheel immediately ----
        stopFlywheel = new ButtonAction(
                new InstantCommand(() -> OpModeBase.INSTANCE.setFlywheelVelocity(0)),
                commandRunner
        );

        // ---- GP2: Manual gate ----
        openGate = new ButtonAction(
                new InstantCommand(() ->
                        OpModeBase.INSTANCE.gateServo.setPosition(Constants.Gate.OPEN_POSITION)
                ), commandRunner
        );
        closeGate = new ButtonAction(
                new InstantCommand(() ->
                        OpModeBase.INSTANCE.gateServo.setPosition(Constants.Gate.CLOSE_POSITION)
                ), commandRunner
        );
    }

    @Override
    protected void runLoop() {

        // =====================================================
        // GAMEPAD 1 — DRIVER
        // =====================================================

        // Bumper strafe — left bumper strafes left, right bumper strafes right
        // These override the left stick X when pressed
        double strafe = -gamepad1.left_stick_x;
        if (gamepad1.left_bumper)  strafe = -1.0;
        if (gamepad1.right_bumper) strafe =  1.0;

        OpModeBase.INSTANCE.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                strafe,
                -gamepad1.right_stick_x,
                false
        );

        // Alliance selection (Start + face button)
        if (gamepad1.start && gamepad1.a) {
            isBlue = true;
            buildActions();
            telemetry.addData("Alliance", "BLUE");
        }
        if (gamepad1.start && gamepad1.b) {
            isBlue = false;
            buildActions();
            telemetry.addData("Alliance", "RED");
        }

        // Snap to shooting position (path + heading)
        snapClose.update(gamepad1.a && !gamepad1.start);
        snapMedium.update(gamepad1.b && !gamepad1.start);
        snapTop.update(gamepad1.x);
        snapFar.update(gamepad1.y);

        // D-pad
        goToHumanPlayer.update(gamepad1.dpad_right);
        pathToPark.update(gamepad1.dpad_left);
        deployKickstand.update(gamepad1.dpad_up);
        retractKickstand.update(gamepad1.dpad_down);

        // =====================================================
        // GAMEPAD 2 — OPERATOR
        // =====================================================

        // Intake — analog, left stick Y
        double intakePower = -gamepad2.left_stick_y;
        if (Math.abs(intakePower) > 0.1) {
            OpModeBase.INSTANCE.intakeMotor.setPower(intakePower);
        } else {
            OpModeBase.INSTANCE.intakeMotor.setPower(0);
        }

        // Hood position
        hoodClose.update(gamepad2.a);
        hoodMedium.update(gamepad2.b);
        hoodTop.update(gamepad2.x);
        hoodFar.update(gamepad2.y);

        // Shoot / stop flywheel
        shoot.update(gamepad2.right_bumper);
        stopFlywheel.update(gamepad2.left_bumper);

        // Manual gate
        openGate.update(gamepad2.left_trigger > 0.5);
        closeGate.update(gamepad2.right_trigger > 0.5);

        // =====================================================
        // TELEMETRY
        // =====================================================
        telemetry.addData("Alliance",          isBlue ? "BLUE" : "RED");
        telemetry.addData("Hood position",     currentPosition);
        telemetry.addData("Flywheel velocity", OpModeBase.INSTANCE.getFlywheelVelocity());
        telemetry.addData("Pose X",            OpModeBase.INSTANCE.follower.getPose().getX());
        telemetry.addData("Pose Y",            OpModeBase.INSTANCE.follower.getPose().getY());
        telemetry.addData("Heading",           Math.toDegrees(
                OpModeBase.INSTANCE.follower.getPose().getHeading()));
        telemetry.update();
    }
}