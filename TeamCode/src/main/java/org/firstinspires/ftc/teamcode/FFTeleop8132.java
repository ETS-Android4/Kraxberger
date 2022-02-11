package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FFTeleop8132", group="Linear Opmode")///This name will be on the drivers phone

public class FFTeleop8132  extends LinearOpMode {/// SkystoneTeleop8132Price(your Team Number and Your Last Name)

    FFHardware8132 robot = new FFHardware8132();/// SkystoneTeleop8132Price(your Team Number and Your Last Name)

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double rail_power=0;
        double duck_power=0;

        double speed_ratio = 1.0;

        robot.init(hardwareMap);  /// Initialize the hardware file

        /// Drive Controls

        telemetry.addData("Say", "Hello Gearhead");  ///Send Telemetry to signify robot waiting
        telemetry.update();

        waitForStart();   //Wait for the game to start (driver presses PLAY)

        while (opModeIsActive()) {// run until the end of the match (driver presses STOP)

            left = -gamepad1.left_stick_y*speed_ratio;// Tank drive
            right = -gamepad1.right_stick_y*speed_ratio;//tank drive

            max = Math.max(Math.abs(left), Math.abs(right)); // Normalize the values so niether exeed +/- 1.0
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            robot.driveMotors(left, right); // Output the safe values to the motor drives.
            //////////////////////////////////////////////////////////////////////////////////////////
            ///Robot Control for Mech
            ///Rail///
            if (gamepad1.x)
                robot.rail_up(0.5);
            else if (gamepad1.b)
                robot.rail_down(0.5);
            else robot.rail_stop();

            ///Duck///
            if (gamepad1.dpad_left)
                robot.duck_up(0.5);
            else if (gamepad1.dpad_right)
                robot.duck_down(0.5);
            else robot.duck_stop();

            ///Claw///
            if (gamepad1.left_bumper)//fast open
                robot.claw_open();
            else if (gamepad1.left_trigger>0.1)//slow open
                robot.claw_close();
            else if (gamepad1.right_bumper)//slow open
                robot.claw_inc();
            else if (gamepad1.right_trigger>0.1)//fast open
                robot.claw_dec();

            ///Gas Pedal for the Robot///
            ///Speed control for robot///

            if (gamepad1.a) { //slow mode
                speed_ratio -= 0.01;
                if (speed_ratio<0.01)
                    speed_ratio=0.1;
            }else if (gamepad1.y){
                speed_ratio += 0.01;
                if (speed_ratio>1)
                    speed_ratio = 1;
            }
            rail_power = robot.rail.getPower();
            duck_power = robot.duck.getPower();

            //Send telemetry message to signify robot running;data for robot
            telemetry.addData("left =", "%.2f (fec=%d,bec=%d)", left,
                    robot.leftfrontMotor.getCurrentPosition(),robot.leftbackMotor.getCurrentPosition());//left motor power
            telemetry.addData("right =", "%.2f (fec=%d,bec=%d)", right,
                    robot.rightfrontMotor.getCurrentPosition(),robot.rightbackMotor.getCurrentPosition());//right motor power
            telemetry.addData("rail  =", "pw=%.2f, enc=%d", rail_power,robot.rail.getCurrentPosition()); // rail power
            telemetry.addData("duck  =", "pw=%.2f, enc=%d", duck_power,robot.duck.getCurrentPosition()); // duck power
            telemetry.addData("claw   =", "%.2f", robot.claw.getPosition());//Claw position
            telemetry.addData("speed ratio =","%.2f", speed_ratio);//shows speed of robot
            telemetry.addData("imu    =", "%.1f", robot.get_heading());
            telemetry.update();

            sleep(50);
        }
    }
}
// end of class  FFTeleOp8132/// (your Team Number 
