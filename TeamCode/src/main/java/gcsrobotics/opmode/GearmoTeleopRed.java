package gcsrobotics.opmode;

import static gcsrobotics.pedroPathing.Constants.Flywheel.VELOCITY_IDLE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import gcsrobotics.commands.FollowPath;
import gcsrobotics.commands.GoToHumanPlayer;
import gcsrobotics.commands.KickstandSubsystem;
import gcsrobotics.commands.Park;
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
    //  Left stick Y        Forward / back
    //  Left stick X        Strafe left / right
    //  Right stick X       Rotation
    //  Left bumper         Strafe LEFT (overrides stick)
    //  Right bumper        Strafe RIGHT (overrides stick)
    //  Left trigger        Strafe left (analog)
    //  Right trigger       Strafe right (analog)
    //  A                   Path to CLOSE shooting position (TODO: needs coordinates)
    //  B                   Path to FAR shooting position (TODO: needs coordinates)
    //  X                   PathToCenter
    //  Y                   PathToFarZone
    //  D-pad Up            Deploy kickstand (hold) / stop on release
    //  D-pad Down          Retract kickstand (hold) / stop on release
    //  D-pad Right         GoToHumanPlayer
    //  D-pad Left          Park
    //  Start + A           Set alliance → BLUE
    //  Start + B           Set alliance → RED
    //
    //  GAMEPAD 2 — OPERATOR
    //  Left stick Y        Intake (forward = in, pull back = unjam)
    //                      Suppressed during shoot sequence
    //  A                   Hood → CLOSE  + flywheel CLOSE (slowest)
    //  X                   Hood → MEDIUM + flywheel MEDIUM
    //  Y                   Hood → TOP    + flywheel TOP
    //  B                   Hood → FAR    + flywheel FAR (fastest)
    //  Right bumper        Shoot at current position
    //  Left bumper         Stop flywheel
    //  Left trigger        Emergency gate open
    //  Right trigger       Emergency gate close
    // ============================================================

    // ---- Alliance ----
    private boolean isBlue = true;

    // ---- Current shooting position (updated by GP2 A/X/Y/B) ----
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

    // ---- GP2 button actions ----
    private ButtonAction hoodClose;
    private ButtonAction hoodMedium;
    private ButtonAction hoodTop;
    private ButtonAction hoodFar;
    private ButtonAction shoot;
    private ButtonAction stopFlywheel;


    // ---- Drive motors ----
    private DcMotorSimple fl;
    private DcMotorSimple fr;
    private DcMotorSimple bl;
    private DcMotorSimple br;
    private double speed = 1.0;

    @Override
    protected void initialize() {
        // Disable Pedro's built-in drive — we control motors directly
        driveMode = false;

        kickstand = new KickstandSubsystem(
                hardwareMap.get(CRServo.class, Constants.Kickstand.MOTOR_NAME)
        );

        // ---- Drive motors ----
        fl = hardwareMap.get(DcMotorSimple.class, "fl");
        fr = hardwareMap.get(DcMotorSimple.class, "fr");
        bl = hardwareMap.get(DcMotorSimple.class, "bl");
        br = hardwareMap.get(DcMotorSimple.class, "br");

        // ---- Close gate on init ----
        OpModeBase.INSTANCE.gateServo.setPosition(Constants.Gate.CLOSE_POSITION);

        buildActions();

        telemetry.addData("Status",   "Initialized");
        telemetry.addData("Alliance", isBlue ? "BLUE" : "RED");
        telemetry.update();
    }

    private void buildActions() {

        // ---- GP1: Path to shooting position ----
        // TODO: Populate ShootingPosition coordinates after field measurement
        // snapClose = new ButtonAction(
        //         () -> new FollowPath(ShootingPosition.CLOSE.poseX,
        //                 ShootingPosition.CLOSE.poseY,
        //                 ShootingPosition.CLOSE.poseHeading),
        //         commandRunner
        // );
        // snapMedium = new ButtonAction(
        //         () -> new FollowPath(ShootingPosition.MEDIUM.poseX,
        //                 ShootingPosition.MEDIUM.poseY,
        //                 ShootingPosition.MEDIUM.poseHeading),
        //         commandRunner
        // );
        snapClose = null;
        snapMedium = null;

        // ---- GP1: X → PathToCenter, Y → PathToFarZone ----
        snapTop = new ButtonAction(
                () -> new gcsrobotics.commands.PathToCenter(isBlue), commandRunner
        );
        snapFar = new ButtonAction(
                () -> new gcsrobotics.commands.PathToFarZone(isBlue), commandRunner
        );

        // ---- GP1: D-pad ----
        goToHumanPlayer = new ButtonAction(
                () -> new GoToHumanPlayer(isBlue), commandRunner
        );
        pathToPark = new ButtonAction(
                () -> new Park(isBlue), commandRunner
        );

        // ---- GP2: Hood position + flywheel speed ----
        // A = CLOSE (slowest), X = MEDIUM, Y = TOP, B = FAR (fastest)
        hoodClose = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.CLOSE;
                            OpModeBase.INSTANCE.setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_CLOSE);
                        }),
                        new SetHoodAngle(ShootingPosition.CLOSE)
                ), commandRunner
        );
        hoodMedium = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.MEDIUM;
                            OpModeBase.INSTANCE.setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_MEDIUM);
                        }),
                        new SetHoodAngle(ShootingPosition.MEDIUM)
                ), commandRunner
        );
        hoodTop = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.TOP;
                            OpModeBase.INSTANCE.setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_TOP);
                        }),
                        new SetHoodAngle(ShootingPosition.TOP)
                ), commandRunner
        );
        hoodFar = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.FAR;
                            OpModeBase.INSTANCE.setFlywheelVelocity(
                                    Constants.Flywheel.VELOCITY_FAR);
                        }),
                        new SetHoodAngle(ShootingPosition.FAR)
                ), commandRunner
        );

        // ---- GP2: Shoot at current position ----
        shoot = new ButtonAction(
                () -> new ShootCurrent(() -> currentPosition), commandRunner
        );

        // ---- GP2: Stop flywheel immediately ----
        stopFlywheel = new ButtonAction(
                () -> new InstantCommand(() -> OpModeBase.INSTANCE.setFlywheelVelocity(0)),
                commandRunner
        );
    }

    @Override
    protected void runLoop() {
        // =====================================================
// LED INDICATOR — Artifact detection
// =====================================================
        boolean objectDetected = OpModeBase.INSTANCE.isBallAtTransfer();
        if (objectDetected) {
            OpModeBase.INSTANCE.ledLight.setPosition(0.5); // Green
        } else {
            OpModeBase.INSTANCE.ledLight.setPosition(0.0); // Off
        }
        // GAMEPAD 1 — DRIVER
        // =====================================================

        // Bumper/trigger strafe overrides stick when pressed
        double horizontal;
        if (gamepad1.left_bumper) {
            horizontal = -1.0;                        // strafe LEFT
        } else if (gamepad1.right_bumper) {
            horizontal = 1.0;                         // strafe RIGHT
        } else {
            double stick = Math.abs(gamepad1.left_stick_x) > 0.1
                    ? gamepad1.left_stick_x : 0;
            double rTrig = Math.abs(gamepad1.right_trigger) > 0.1
                    ? gamepad1.right_trigger : 0;
            double lTrig = Math.abs(gamepad1.left_trigger) > 0.1
                    ? gamepad1.left_trigger : 0;
            horizontal = stick + rTrig - lTrig;
        }

        double forward = Math.abs(gamepad1.left_stick_y) > 0.1
                ? -gamepad1.left_stick_y : 0;
        double pivot = Math.abs(gamepad1.right_stick_x) > 0.1
                ? gamepad1.right_stick_x : 0;

        // Standard mecanum formula
        fl.setPower(speed * ( forward + horizontal + pivot));
        fr.setPower(speed * ( forward - horizontal - pivot));
        bl.setPower(speed * ( forward - horizontal + pivot));
        br.setPower(speed * ( forward + horizontal - pivot));

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

        // Snap to shooting position
        // TODO: Remove null checks once ShootingPosition coordinates are measured
        if (snapClose != null)  snapClose.update(gamepad1.a && !gamepad1.start);
        if (snapMedium != null) snapMedium.update(gamepad1.b && !gamepad1.start);
        snapTop.update(gamepad1.x);
        snapFar.update(gamepad1.y);

        // D-pad path commands
        goToHumanPlayer.update(gamepad1.dpad_right);
        pathToPark.update(gamepad1.dpad_left);

        // Kickstand — runs while held, stops on release
        if (gamepad1.dpad_up) {
            kickstand.deploy();
        } else if (gamepad1.dpad_down) {
            kickstand.retract();
        } else {
            kickstand.stop();
        }

        // =====================================================
        // GAMEPAD 2 — OPERATOR
        // =====================================================

        // Intake — suppressed during shoot sequence to prevent override
        if (commandRunner.isFinished()) {
            double intakePower = gamepad2.left_stick_y;
            if (Math.abs(intakePower) > 0.1) {
                OpModeBase.INSTANCE.intakeMotor.setPower(intakePower);
            } else {
                OpModeBase.INSTANCE.intakeMotor.setPower(0);
            }
        }
        // Hood position + flywheel speed
        // A = CLOSE (slowest), X = MEDIUM, Y = TOP, B = FAR (fastest)
        hoodClose.update(gamepad2.a);
        hoodMedium.update(gamepad2.x);
        hoodTop.update(gamepad2.y);
        hoodFar.update(gamepad2.b);

        // Shoot / stop flywheel
        shoot.update(gamepad2.right_bumper);
        stopFlywheel.update(gamepad2.left_bumper);

        double manualFlywheel = -gamepad2.right_stick_y;
        if (Math.abs(manualFlywheel) > 0.1) {
            // Map stick range (-1 to 1) to RPM range (0 to 6000)
            double targetRPM = manualFlywheel * 6000.0;
            OpModeBase.INSTANCE.setFlywheelVelocity(targetRPM);
        }
        if (gamepad2.dpad_up) {
            OpModeBase.INSTANCE.setFlywheelVelocity(Constants.Flywheel.VELOCITY_IDLE);
        }
        // TEMPORARY TEST — GP2 dpad down = full power flywheel
        if (gamepad2.dpad_down) {
            OpModeBase.INSTANCE.flywheelLeft.setPower(1.0);
            OpModeBase.INSTANCE.flywheelRight.setPower(1.0);
        }
        // Emergency manual gate override
        if (gamepad2.left_trigger > 0.5) {
            OpModeBase.INSTANCE.gateServo.setPosition(Constants.Gate.OPEN_POSITION);
        } else if (gamepad2.right_trigger > 0.5) {
            OpModeBase.INSTANCE.gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
        }

        // =====================================================
        // TELEMETRY
        // =====================================================
        telemetry.addData("Hood position",   currentPosition);
        telemetry.addData("Flywheel RPM",    OpModeBase.INSTANCE.getFlywheelVelocity());
        telemetry.addData("Shooting active", !commandRunner.isFinished());
        telemetry.addData("Pose X",          OpModeBase.INSTANCE.follower.getPose().getX());
        telemetry.addData("Pose Y",          OpModeBase.INSTANCE.follower.getPose().getY());
        telemetry.addData("Heading",         Math.toDegrees(
                OpModeBase.INSTANCE.follower.getPose().getHeading()));
        telemetry.update();
        telemetry.addData("FL ticks/sec", OpModeBase.INSTANCE.flywheelLeft.getVelocity());
        telemetry.addData("FR ticks/sec", OpModeBase.INSTANCE.flywheelRight.getVelocity());
        telemetry.addData("Artifact Detected", OpModeBase.INSTANCE.isBallAtTransfer());
    }
}