//Red has not been tested//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@Autonomous (name="StorageParkRed8188", group="Linear OpMode")

public class ParkWareAutoRed8188 extends LinearOpMode {
    
FFHardware8188 robot = new FFHardware8188();

    @Override
public void runOpMode() throws InterruptedException {
    double left;
    double right;
    double drive;
    double turn;
    double max;
    //double rail_power=0;
    robot.init(hardwareMap); /// Initialize the hardware file.
    
    telemetry.addData("Driver,", "press Start for Auto8188 run...");  ///Send telemetry message to signify robot waiting
    telemetry.update();
    
waitForStart(); //Wait for the game to start (driver presses PLAY)

//Robot Turns //45 postive robot turns left/ right turn would be -45. 1-179 for turning for left, -1-179 turning right
//Power 0.1-1, more power less precision

//Driving

//robot.duck_up(20.0);
//sleep(2700);

robot.straightIn(0.50, 18.50); // First number power and second number how far 
sleep(500);

robot.turn(-35.00,0.50);
sleep(500);

robot.straightIn(-0.50, 7.50); // First number power and second number how far 
sleep(500);

robot.turn(-25.00,0.50);
sleep(500);

robot.straightIn(-0.50, 5.50); // First number power and second number how far 
sleep(500);

robot.turn(-15.00,0.50);
sleep(500);

robot.straightIn(-0.50, 10.50); // First number power and second number how far 
sleep(500);

robot.duck_up(0);
sleep(2850);

robot.straightIn(0.50, 3.50); // First number power and second number how far 
sleep(500);

robot.turn(120.00,0.50);
sleep(500);

robot.straightIn(0.50, 5.50); // First number power and second number how far 
sleep(500);

robot.turn(-20.00,0.50);
sleep(500);

robot.straightIn(0.50, 10.50); // First number power and second number how far 
sleep(500);

//robot.straightIn(-0.25, 20.0);/// First number is turning, second number is how long
//sleep(200);

//robot.straightIn(0.25, 20.0); // foward 50 inches
//sleep(200);

//robot.straightIn(0.25, 5.0);
//sleep(500);

//robot.turn(20, 0.25);
//sleep(200);

//Move Platform

//robot.turn(-80.00,0.50);
//sleep(500);

//robot.straightIn(0.50, 17.00); // First number power and second number how far 
//sleep(500);

//robot.turn(10.00,0.50);
//sleep(500);

//robot.straightIn(0.50, 1.25); // First number power and second number how far 
//sleep(500);

//robot.turn(17.00,0.50);
//(500);

//robot.straightIn(0.50, 1.00); // First number power and second number how far 
//sleep(500);

//robot.turn(7.00,0.50);
//sleep(500);

//robot.duck_up(20.0);
//sleep(2700);

//robot.straightIn(0.50, 1.50); // First number power and second number how far 
//sleep(500);

//robot.turn(10.00,0.50);
//sleep(500);

//robot.straightIn(0.50, 4.50); // First number power and second number how far 
//sleep(500);

//robot.turn(15.00,0.50);
//sleep(500);

//robot.straightIn(0.50, 10.50); // First number power and second number how far 
//sleep(500);

//robot.straightIn(-0.25, 35.0);
//sleep(500);

//robot.turn(-30.0, -0.5);
//sleep (500);

//robot.straightIn(-0.5, 10.0);
//sleep(500);

//robot.turn(-38.0, -0.5);
//sleep(500);

//robot.straightIn(-0.5, 5.0);
//sleep(500);

//robot. actuator_open();

}

//end runOpMode()

} // end class 
