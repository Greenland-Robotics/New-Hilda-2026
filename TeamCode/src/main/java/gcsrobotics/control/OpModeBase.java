package gcsrobotics.control;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.CommandRunner;

public abstract class OpModeBase extends LinearOpMode {
    public static volatile OpModeBase INSTANCE;
    public Follower follower;
    protected CommandRunner commandRunner;

    protected abstract void initInternal();
    protected abstract void loopInternal();

    @Override
    public void runOpMode() {
        initHardware();
        follower = Constants.createFollower(hardwareMap);
        initInternal();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            loopInternal();
        }
    }

    /*eg.*/ public DcMotorEx flywheel;
    /*eg.*/ public DcMotorEx intake;
    /*eg.*/ public Servo servo;
    /*eg.*/ public CRServo crServo;
    private void initHardware(){
        /* eg.*/ flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        /* eg.*/ intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    protected double getX() {return follower.getPose().getX();}
    protected double getY() {return follower.getPose().getY();}
    protected double getHeading() {return follower.getPose().getHeading();}

}
