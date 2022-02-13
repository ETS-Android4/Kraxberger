package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@Disabled
@Autonomous(name="Warehouse8132", group="Linear OpMode")

public class ParkWareAuto8132 extends LinearOpMode {

  FFHardware8132 robot = new FFHardware8132();

  @Override
  public void runOpMode() throws InterruptedException {
    double left;
    double right;
    double drive;
    double turn;
    double max;

    //double rail_power=0;
    robot.init(hardwareMap);  /// Initialize the hardware file.

    telemetry.addData("Driver,", "press Start for Auto8132  run...");   ///Send telemetry message to signify robot waiting
    telemetry.update();

    waitForStart();  // Wait for the game to start (driver presses PLAY)

    // move rail to up position using 0.2 power
    robot.rail_to_position(0.2, robot.rail_up_pos);

    //Robot Turns //45 postive robot turns left/ right turn would be -45.  1-179 for turning for left , -1-179 turning right
    //Power  0.1-1, more power less precision

    //Driving

    //robot.rail_down(-0.1);
    //sleep(100);


    robot.straightIn(0.25, 31.0); // First number power and second number how far
    sleep(200);

    robot.straightIn(-0.25, 20.0);/// First number is turning, second number is how long
    sleep(200);

    robot.turn(20, 0.25); // forward 50 inches
    sleep(200);

    robot.straightIn(-0.25, 5.0);
    sleep(500);

    robot.turn(20, 0.25);
    sleep(200);

    //Move Platform

    robot.turn(50.0, 0.25);
    sleep(500);

    robot.straightIn(-0.25, 35.0);
    sleep(500);

    //robot.turn(-30.0, -0.5);
    //sleep(500);

    //robot.straightIn(-0.5, 10.0);
    //sleep(500);

    //robot.turn(-38.0, -0.5);
    //sleep(500);

    //robot.straightIn(-0.5, 5.0);
    //sleep(500);

  }

//end runOpMode()

} // end class 
