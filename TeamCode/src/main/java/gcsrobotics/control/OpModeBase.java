package gcsrobotics.control;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.CommandRunner;

public abstract class OpModeBase extends LinearOpMode {
    public static volatile OpModeBase INSTANCE;
    public Follower follower;
    protected CommandRunner commandRunner;

    // goBILDA 6000 RPM Yellow Jacket — 28 ticks per revolution
    private static final double TICKS_PER_REV = 28.0;
    public double currentTargetRPM = 0.0;
    public boolean flywheelFullPower = false;

    // ---- Intake ----
    public DcMotorEx intakeMotor;

    // ---- Shooter ----
    public DcMotorEx flywheelLeft;
    public DcMotorEx flywheelRight;
    public Servo hoodServo;
    public Servo gateServo;
    public CRServo kickstand;
    public Servo ledLight;

    // ---- Odometry ----
    public GoBildaPinpointDriver odo;

    // ---- Sensors ----
    // Only intakeSensor is actively polled — transfer and shot sensors
    // are initialized but not read to reduce loop latency
    public DigitalChannel transferSensor;
    public DigitalChannel intakeSensor;
    public DigitalChannel shotSensor;

    protected abstract void initInternal();
    protected abstract void loopInternal();

    @Override
    public void runOpMode() {
        INSTANCE = this;

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        initHardware();
        follower = Constants.createFollower(hardwareMap);
        initInternal();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            loopInternal();
            updateFlywheel();
            telemetry.update();
        }
    }

    private void initHardware() {
        // ---- Odometry ----
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // ---- Intake ----
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // ---- Flywheels ----
        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelRight.setDirection(DcMotorEx.Direction.REVERSE);

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
        kickstand = hardwareMap.get(CRServo.class, "kickstand");

        // ---- Sensors ----
        // All three initialized to avoid hardware map exceptions,
        // but only intakeSensor is actively polled
        transferSensor = hardwareMap.get(DigitalChannel.class, "transferSensor");
        transferSensor.setMode(DigitalChannel.Mode.INPUT);

        intakeSensor = hardwareMap.get(DigitalChannel.class, "intakeSensor");
        intakeSensor.setMode(DigitalChannel.Mode.INPUT);

        shotSensor = hardwareMap.get(DigitalChannel.class, "shotSensor");
        shotSensor.setMode(DigitalChannel.Mode.INPUT);

        // ---- LED ----
        ledLight = hardwareMap.get(Servo.class, "ledLight");
        ledLight.setPosition(Constants.LED.OFF);
    }

    // ---- Pose Helpers ----
    protected double getX()       { return follower.getPose().getX(); }
    protected double getY()       { return follower.getPose().getY(); }
    protected double getHeading() { return follower.getPose().getHeading(); }

    // ---- Flywheel Helpers (all in RPM) ----
    public void setFlywheelVelocity(double rpm) {
        currentTargetRPM = rpm;
    }

    public void updateFlywheel() {
        if (flywheelFullPower) {          // ← add these three lines
            flywheelLeft.setPower(1.0);
            flywheelRight.setPower(1.0);
            return;
        }

        if (currentTargetRPM <= 0) {
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
            return;
        }

        double actualRPM  = getFlywheelVelocity();
        double threshold  = currentTargetRPM * Constants.Flywheel.BANG_BANG_THRESHOLD;

        if (actualRPM < threshold) {
            flywheelLeft.setPower(Constants.Flywheel.BANG_BANG_POWER);
            flywheelRight.setPower(Constants.Flywheel.BANG_BANG_POWER);
        } else {
            double voltage     = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double ticksPerSec = currentTargetRPM * TICKS_PER_REV / 60.0 * (12.0 / voltage);
            flywheelLeft.setVelocity(ticksPerSec);
            flywheelRight.setVelocity(ticksPerSec);
        }
    }

    public void setFlywheelPower(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    public double getFlywheelVelocity() {
        double avgTicks = (flywheelLeft.getVelocity() + flywheelRight.getVelocity()) / 2.0;
        return avgTicks * 60.0 / TICKS_PER_REV;
    }

    // ---- Sensor Helpers ----
    public boolean isBallAtTransfer() {
        return false;  // disabled — not polled to reduce loop latency
    }

    public boolean isBallAtIntake() {
        return intakeSensor.getState();
    }

    public boolean isBallAtShot() {
        return false;  // disabled — not polled to reduce loop latency
    }

    // Returns count of balls detected (intake sensor only)
    public int getBallCount() {
        return intakeSensor.getState() ? 1 : 0;
    }

    // Updates LED based on intake sensor only:
    //   ball present → GREEN
    //   no ball      → OFF
    public void updateLED() {
        if (intakeSensor.getState()) {
            ledLight.setPosition(Constants.LED.SOLID_GREEN);
        } else {
            ledLight.setPosition(Constants.LED.OFF);
        }
    }
}