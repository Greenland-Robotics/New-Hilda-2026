package gcsrobotics.control;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.CommandRunner;

public abstract class OpModeBase extends LinearOpMode {
    public static volatile OpModeBase INSTANCE;
    public Follower follower;
    protected CommandRunner commandRunner;

    // ---- Intake ----
    public DcMotorEx intakeMotor;

    // ---- Shooter ----
    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;
    public Servo hoodServo;
    public Servo gateServo;

    // ---- Sensors ----
    public DigitalChannel shootSensor;          // FIX 5: goBILDA laser in digital/beam-break mode
    public DistanceSensor transferSensor;       // TODO: confirm sensor type for transfer

    protected abstract void initInternal();
    protected abstract void loopInternal();

    @Override
    public void runOpMode() {
        INSTANCE = this;
        initHardware();
        follower = Constants.createFollower(hardwareMap);
        initInternal();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            loopInternal();
        }
    }

    private void initHardware() {
        // ---- Intake ----
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD); // TODO: confirm direction

        // ---- Flywheels ----
        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setDirection(DcMotorEx.Direction.FORWARD);  // TODO: confirm direction
        flywheelRight.setDirection(DcMotorEx.Direction.REVERSE); // TODO: confirm direction

        // FIX 4: Set RUN_USING_ENCODER mode and apply PIDF coefficients
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setVelocityPIDFCoefficients(
                Constants.Flywheel.PIDF_P,
                Constants.Flywheel.PIDF_I,
                Constants.Flywheel.PIDF_D,
                Constants.Flywheel.PIDF_F
        );
        flywheelRight.setVelocityPIDFCoefficients(
                Constants.Flywheel.PIDF_P,
                Constants.Flywheel.PIDF_I,
                Constants.Flywheel.PIDF_D,
                Constants.Flywheel.PIDF_F
        );

        // ---- Servos ----
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        gateServo = hardwareMap.get(Servo.class, "gateServo");

        // ---- Sensors ----
        shootSensor    = hardwareMap.get(DigitalChannel.class, "shootSensor");
        shootSensor.setMode(DigitalChannel.Mode.INPUT); // configure as input for beam-break
        transferSensor = hardwareMap.get(DistanceSensor.class, "transferSensor"); // TODO: confirm type
    }

    // ---- Pose Helpers ----
    protected double getX()       { return follower.getPose().getX(); }
    protected double getY()       { return follower.getPose().getY(); }
    protected double getHeading() { return follower.getPose().getHeading(); }

    // ---- Flywheel Helpers ----
    // FIX 3: Added setFlywheelVelocity() for PIDF-controlled velocity targeting
    public void setFlywheelVelocity(double radPerSec) {
        flywheelLeft.setVelocity(radPerSec, AngleUnit.RADIANS);
        flywheelRight.setVelocity(radPerSec, AngleUnit.RADIANS);
    }

    public void setFlywheelPower(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    // FIX 2: getVelocity() now uses AngleUnit.RADIANS to match Constants.Flywheel targets
    public double getFlywheelVelocity() {
        return (flywheelLeft.getVelocity(AngleUnit.RADIANS) +
                flywheelRight.getVelocity(AngleUnit.RADIANS)) / 2.0;
    }

    // ---- Shoot Sensor Helper ----
    // goBILDA laser in digital mode: LOW = beam broken = ball present
    public boolean isBallAtShooter() {
        return !shootSensor.getState();
    }
}