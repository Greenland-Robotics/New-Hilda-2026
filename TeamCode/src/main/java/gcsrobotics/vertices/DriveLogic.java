package gcsrobotics.vertices;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import gcsrobotics.control.OpModeBase;

public abstract class DriveLogic extends OpModeBase {
    public DcMotorSimple fl;
    public DcMotorSimple fr;
    public DcMotorSimple bl;
    public DcMotorSimple br;
    public double speed;


    public void implementDriveLogic() {
        fl = hardwareMap.get(DcMotorSimple.class, "fl");
        fr = hardwareMap.get(DcMotorSimple.class, "fr");
        bl = hardwareMap.get(DcMotorSimple.class, "bl");
        br = hardwareMap.get(DcMotorSimple.class, "br");

        double horizontal;
        // Horizontal Lock
        if (gamepad1.right_bumper) {
            horizontal = 0;
        } else {
            // Joystick and trigger combined horizontal control
            double rightTriggerDeadZone = Math.abs(gamepad1.right_trigger) > 0.1 ? gamepad1.right_trigger : 0;
            double leftTriggerDeadZone = Math.abs(gamepad1.left_trigger) > 0.1 ? gamepad1.left_trigger : 0;
            double stickDeadZone = Math.abs(gamepad1.left_stick_x) > 0.1 ? gamepad1.left_stick_x : 0;
            horizontal = -stickDeadZone - rightTriggerDeadZone + leftTriggerDeadZone;
        }

        double pivot = Math.abs(gamepad1.right_stick_x) > 0.1 ? gamepad1.right_stick_x : 0;

        // Use consistent naming: forward = X axis, strafe = Y axis (GoBILDA convention)
        double forward = -gamepad1.left_stick_y;  // Forward/back (X in GoBILDA)
        double strafe = -horizontal;               // Left/right (Y in GoBILDA)

//        if (fieldCentric) {
//            // Field-centric compensation using GoBILDA convention
//            double headingRad = Math.toRadians(odo.getAngle());
//            double tempForward = forward * Math.cos(headingRad) - strafe * Math.sin(headingRad);
//            double tempStrafe = forward * Math.sin(headingRad) + strafe * Math.cos(headingRad);
//            forward = tempForward;
//            strafe = tempStrafe;
//        }

        // Mecanum drive motor calculations
        // Forward: all motors same direction, Strafe: diagonal pattern
        fl.setPower(speed * (pivot + forward + strafe));
        fr.setPower(speed * (-pivot + forward - strafe));
        bl.setPower(speed * (pivot + forward - strafe));
        br.setPower(speed * (-pivot + forward + strafe));
    }

}
