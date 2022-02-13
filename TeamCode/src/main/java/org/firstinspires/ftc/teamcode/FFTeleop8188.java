package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.NavigableMap;

@TeleOp(name="FFTeleop8188", group="Linear Opmode")///This name will be on the drivers phone

public class FFTeleop8188 extends LinearOpMode {
    
FFHardware8188 robot = new FFHardware8188();

@Override
public void runOpMode() {
    double left;
    double right;
    double drive;
    double turn;
    double max;
    double duck_power=0;
    double rail_power=0;
    double speed_ratio = 1.0;

    robot.init(hardwareMap); /// Initialize the hardware file

/// Drive Controls

    telemetry.addData("Say", "Hello Gearhead"); ///Send telemetry to signify robot waiting
    telemetry.update();

    waitForStart();  //Wait for the game to start (Driver presses PLAY)

    while (opModeIsActive()) {// run until the end of the match (driver presses STOP)

        left = -gamepad1.left_stick_y*speed_ratio;// Tank drive
        right = -gamepad1.right_stick_y*speed_ratio;//tank drive

        max = Math.max(Math.abs(left), Math.abs(right)); // Normalize the values soniether exeed +/- 1.0
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        robot.driveMotors(left, right); // Output the safe values to the motor drives.
        /////////////////////////////////////////////////////////////////////////////////////////

        ///Robot Control for Mech

        ///Rail///
        if (gamepad1.dpad_left)
            robot.duck_up(gamepad1.left_trigger);

        else if (gamepad1.dpad_right)
            robot.duck_down(gamepad1.left_trigger);

        else robot.duck_stop();

        ///Rail///
        if (gamepad1.x)
            robot.rail_up(1.0);
        else if (gamepad1.b)
            robot.rail_down(1.0);
        else {
            if (robot.rail.getMode()!= DcMotor.RunMode.RUN_TO_POSITION) {
                robot.rail_stop();
            }
        }

///Servo Hook///-One servo by it self
///if (gamepad1.dpad_left)
///robot.hook_close();
///else if (gamepad1.dpad_right)
///robot.hook_open();

///Claw///
//        if (gamepad1.left_bumper)//fast open
//            robot.claw_open();
//        else if (gamepad1.left_trigger>0.1 && !gamepad1.dpad_left)//slow open
//            robot.claw_close();
        if (gamepad1.left_bumper) {
            if (gamepad1.right_bumper)//slow open
                robot.claw_inc();
            else if (gamepad1.right_trigger > 0.1)//fast open
                robot.claw_dec();

            // combo buttons for rail positions
            if (gamepad1.y) {
                robot.rail_to_position(0.5, robot.RAIL_LEVEL_3);
            } else if (gamepad1.b) {
                robot.rail_to_position(0.5, robot.RAIL_LEVEL_2);
            } else if (gamepad1.a) {
                robot.rail_to_position(0.5, robot.RAIL_INIT);
            }
        } else if (gamepad1.right_bumper) {
            robot.clawToggle();
        }


///Gas Pedal for the Robot///
///Speed control for robot///

        if (gamepad1.a) { //slow mode
            speed_ratio -= 0.01;
            if (speed_ratio<0.01)
                speed_ratio=0.1;
        }else if (gamepad1.y) {
            speed_ratio += 0.01;
            if (speed_ratio>1)
                speed_ratio = 1;
        }

        //Send telemetry message to signify robot running;data for robot
        telemetry.addData("left =", "%.2f (fec=%d,bec=%d", left,
                robot.leftfrontMotor.getCurrentPosition(),robot.leftbackMotor.getCurrentPosition());//left motor power
        telemetry.addData("right =", "%.2f, (fec=%d,bec=%d", right,
                robot.rightfrontMotor.getCurrentPosition(),robot.rightbackMotor.getCurrentPosition());//right motor power
        telemetry.addData("rail  =", "pw=%.2f, enc=%d", rail_power,robot.rail.getCurrentPosition()); // duck power
        telemetry.addData("duck  =", "pw=%.2f, enc=%d", robot.duck.getPower(),robot.duck.getCurrentPosition()); // duck power
        telemetry.addData("claw  =", "%.2f", robot.claw.getPosition());//Claw position
        telemetry.addData("speed ratio =","%.2f", speed_ratio);//shows speed of robot
        telemetry.addData("imu    =","%.1f", robot.get_heading());
        telemetry.update();

        sleep(50);
    }
}
}
// end of class SkystoneTeleOp8132
