package gcsrobotics.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import gcsrobotics.commands.DeployKickstandCommand;
import gcsrobotics.commands.GoToGate;
import gcsrobotics.commands.GoToHumanPlayer;
import gcsrobotics.commands.KickstandSubsystem;
import gcsrobotics.commands.Park;
import gcsrobotics.commands.PathToCenter;
import gcsrobotics.commands.PathToFarZone;
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

@TeleOp(name = "Hilda TeleOp — NH Premier", group = "Hilda")
public class HildaTeleOp extends TeleOpBase {

    // ============================================================
    //  GAMEPAD 1 — DRIVER
    //  A                   Drive speed 25%
    //  B                   Drive speed 50%
    //  X                   Drive speed 75%
    //  Y                   Drive speed 100%
    //  Left bumper         GoToGate (alliance-aware)
    //  Right bumper        GoToHumanPlayer (alliance-aware)
    //  Left trigger        PathToCenter (alliance-aware)
    //  Right trigger       Park (alliance-aware)
    //  D-pad Up            Deploy kickstand
    //  D-pad Down          Retract kickstand
    //  D-pad Right         PathToFarZone (alliance-aware)
    //  Start + A           Set alliance → BLUE
    //  Start + B           Set alliance → RED
    //
    //  GAMEPAD 2 — OPERATOR
    //  Left stick Y        Intake (forward = in, pull back = unjam)
    //  Left bumper         Snap hood → CLOSE
    //  Right bumper        Snap hood → MEDIUM
    //  Left trigger        Snap hood → TOP
    //  Right trigger       Snap hood → FAR
    //  A                   Shoot at current position
    //  B                   Shoot at current position (backup)
    //  X                   Manual gate open
    //  Y                   Manual gate close
    // ============================================================

    // ---- Alliance ----
    private boolean isBlue = true;

    // ---- Drive speed multiplier ----
    private double driveSpeed = 1.0;

    // ---- Current shooting position (updated by GP2 bumpers/triggers) ----
    private ShootingPosition currentPosition = ShootingPosition.CLOSE;

    // ---- Kickstand ----
    private KickstandSubsystem kickstand;

    // ---- GP1 button actions ----
    private ButtonAction goToGate;
    private ButtonAction goToHumanPlayer;
    private ButtonAction pathToCenter;
    private ButtonAction pathToPark;
    private ButtonAction pathToFarZone;
    private ButtonAction deployKickstand;
    private ButtonAction retractKickstand;

    // ---- GP2 button actions ----
    private ButtonAction snapClose;
    private ButtonAction snapMedium;
    private ButtonAction snapTop;
    private ButtonAction snapFar;
    private ButtonAction shootA;
    private ButtonAction shootB;
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
        telemetry.addData("Speed",    (int)(driveSpeed * 100) + "%");
        telemetry.update();
    }

    // Called once at init and again any time alliance changes
    private void buildActions() {

        // ---- GP1: Path commands (all alliance-aware) ----
        // NOTE: these capture isBlue at build time. buildActions() is
        // called again on Start+A / Start+B so they stay current.
        goToGate = new ButtonAction(
                new GoToGate(isBlue), commandRunner
        );
        goToHumanPlayer = new ButtonAction(
                new GoToHumanPlayer(isBlue), commandRunner
        );
        pathToCenter = new ButtonAction(
                new PathToCenter(isBlue), commandRunner
        );
        pathToPark = new ButtonAction(
                new Park(isBlue), commandRunner
        );
        pathToFarZone = new ButtonAction(
                new PathToFarZone(isBlue), commandRunner
        );

        // ---- GP1: Kickstand ----
        deployKickstand = new ButtonAction(
                new DeployKickstandCommand(kickstand), commandRunner
        );
        retractKickstand = new ButtonAction(
                new RetractKickstandCommand(kickstand), commandRunner
        );

        // ---- GP2: Hood snap (sets position and updates currentPosition) ----
        snapClose = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> currentPosition = ShootingPosition.CLOSE),
                        new SetHoodAngle(ShootingPosition.CLOSE)
                ), commandRunner
        );
        snapMedium = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> currentPosition = ShootingPosition.MEDIUM),
                        new SetHoodAngle(ShootingPosition.MEDIUM)
                ), commandRunner
        );
        snapTop = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> currentPosition = ShootingPosition.TOP),
                        new SetHoodAngle(ShootingPosition.TOP)
                ), commandRunner
        );
        snapFar = new ButtonAction(
                new SeriesCommand(
                        new InstantCommand(() -> currentPosition = ShootingPosition.FAR),
                        new SetHoodAngle(ShootingPosition.FAR)
                ), commandRunner
        );

        // ---- GP2: Shoot at current position ----
        // ShootCurrent resolves currentPosition at button-press time via supplier
        shootA = new ButtonAction(
                new ShootCurrent(() -> currentPosition), commandRunner
        );
        shootB = new ButtonAction(
                new ShootCurrent(() -> currentPosition), commandRunner
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

        // Drive speed presets
        if (gamepad1.a) driveSpeed = 0.25;
        if (gamepad1.b) driveSpeed = 0.50;
        if (gamepad1.x) driveSpeed = 0.75;
        if (gamepad1.y) driveSpeed = 1.00;

        // Apply drive speed scaling — TeleOpBase already sends raw sticks to
        // follower.setTeleOpDrive(), so we override here with scaled values
        OpModeBase.INSTANCE.follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y  * driveSpeed,
                -gamepad1.left_stick_x  * driveSpeed,
                -gamepad1.right_stick_x * driveSpeed,
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

        // Path commands
        goToGate.update(gamepad1.left_bumper);
        goToHumanPlayer.update(gamepad1.right_bumper);
        pathToCenter.update(gamepad1.left_trigger > 0.5);
        pathToPark.update(gamepad1.right_trigger > 0.5);
        pathToFarZone.update(gamepad1.dpad_right);

        // Kickstand
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

        // Hood snap
        snapClose.update(gamepad2.left_bumper);
        snapMedium.update(gamepad2.right_bumper);
        snapTop.update(gamepad2.left_trigger > 0.5);
        snapFar.update(gamepad2.right_trigger > 0.5);

        // Shoot
        shootA.update(gamepad2.a);
        shootB.update(gamepad2.b);

        // Manual gate
        openGate.update(gamepad2.x);
        closeGate.update(gamepad2.y);

        // =====================================================
        // TELEMETRY
        // =====================================================
        telemetry.addData("Alliance",          isBlue ? "BLUE" : "RED");
        telemetry.addData("Drive speed",       (int)(driveSpeed * 100) + "%");
        telemetry.addData("Hood position",     currentPosition);
        telemetry.addData("Flywheel velocity", OpModeBase.INSTANCE.getFlywheelVelocity());
        telemetry.addData("Pose X",            OpModeBase.INSTANCE.follower.getPose().getX());
        telemetry.addData("Pose Y",            OpModeBase.INSTANCE.follower.getPose().getY());
        telemetry.addData("Heading",           Math.toDegrees(
                OpModeBase.INSTANCE.follower.getPose().getHeading()));
        telemetry.update();
    }
}