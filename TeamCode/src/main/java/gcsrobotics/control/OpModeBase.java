package gcsrobotics.control;

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
    // ---- Sensors ----
// goBILDA laser sensors in digital/beam-break mode — LOW = beam broken = ball present
    public DigitalChannel transferSensor;                   // live — wired at transfer
// public DigitalChannel intakeSensor;                  // not yet wired
// public DigitalChannel shotSensor;                    // not yet wired

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
        // ---- Odometry ----
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

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

        // RUN_USING_ENCODER enables velocity PIDF control
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

        // ---- Sensors (commented out until wired) ----
        // shootSensor = hardwareMap.get(DigitalChannel.class, "shootSensor");
        // shootSensor.setMode(DigitalChannel.Mode.INPUT);
        // ---- Sensors ----
        transferSensor = hardwareMap.get(DigitalChannel.class, "transferSensor");
        transferSensor.setMode(DigitalChannel.Mode.INPUT);
// intakeSensor = hardwareMap.get(DigitalChannel.class, "intakeSensor");
// intakeSensor.setMode(DigitalChannel.Mode.INPUT);
// shotSensor = hardwareMap.get(DigitalChannel.class, "shotSensor");
// shotSensor.setMode(DigitalChannel.Mode.INPUT);

// ---- LED ----
        ledLight = hardwareMap.get(Servo.class, "ledLight");
    }

    // ---- Pose Helpers ----
    protected double getX()       { return follower.getPose().getX(); }
    protected double getY()       { return follower.getPose().getY(); }
    protected double getHeading() { return follower.getPose().getHeading(); }

    // ---- Flywheel Helpers (all in RPM) ----
    public void setFlywheelVelocity(double rpm) {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double ticksPerSec = rpm * TICKS_PER_REV / 60.0 * (12.0 / voltage);
        flywheelLeft.setVelocity(ticksPerSec);
        flywheelRight.setVelocity(ticksPerSec);
    }

    public void setFlywheelPower(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    public double getFlywheelVelocity() {
        double avgTicks = (flywheelLeft.getVelocity() + flywheelRight.getVelocity()) / 2.0;
        return avgTicks * 60.0 / TICKS_PER_REV; // returns RPM
    }

    // ---- Shoot Sensor Helper ----
    // goBILDA laser in digital mode: LOW = beam broken = ball present
    // ---- Sensor Helpers ----
// goBILDA laser in digital mode: LOW = beam broken = ball present
    public boolean isBallAtTransfer() { return !transferSensor.getState(); }
// public boolean isBallAtIntake()  { return !intakeSensor.getState();  }
// public boolean isBallAtShooter() { return !shotSensor.getState();    }
}