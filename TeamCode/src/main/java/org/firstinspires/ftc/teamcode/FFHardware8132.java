package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Disabled

public class FFHardware8132 {  /// SkystoneHard8132Price(your Team Number and Your Last Name)

// ------------------------------
// All variables define here 
// ------------------------------

    HardwareMap hwMap = null;  /// Motors
    public DcMotor leftfrontMotor = null;
    public DcMotor rightfrontMotor = null;
    public DcMotor leftbackMotor = null;
    public DcMotor rightbackMotor = null;

    public DcMotor rail;
    int rail_up_pos = 100;
    int rail_down_pos = 0;

    public DcMotor duck;

    Servo claw = null;
    double claw_init =0.98;//makes adjustments to servo
    double claw_close = 0.72; //makes adjustments to servo
    double claw_open = 0.98; //makes adjustments to servo

    // The IMU sensor detects which way the robot is going
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // TensorFlow
    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    // To figure rotation 4in wheel x 3.14=12.56 divide that by 1120.
    //(The number 1120 can be found the website Andy mark under motors-encoders)
    //==89.7==89 clicks on the encoder = 1 Inch

    final static double COUNT_PER_ROTATION_40 = 1120; // neverest-40
    final static double COUNT_PER_INCH = COUNT_PER_ROTATION_40 / 12.56; // assume 4_inch wheels

    public FFHardware8132(){}/// SkystoneHard8132Price(your Team Number and Your Last Name)

    public void init(HardwareMap ahwMap){

    //----------------------------------------------------------------------
    // All hardware objects (motor, servo, sensors) initialize inside init()
    //----------------------------------------------------------------------

        hwMap = ahwMap;

        ///Motors are used to run the robot

        leftfrontMotor = hwMap.dcMotor.get("lf_drive");
        rightfrontMotor = hwMap.dcMotor.get("rf_drive");
        leftbackMotor = hwMap.dcMotor.get("lb_drive");
        rightbackMotor = hwMap.dcMotor.get("rb_drive");

        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftfrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftbackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightbackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftfrontMotor.setPower(0);
        rightfrontMotor.setPower(0);
        leftbackMotor.setPower(0);
        rightbackMotor.setPower(0);

        /* reset motor encoders */
        leftfrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ///Mechanical Devices, arms, claws, shooters

        ///rail
        rail = hwMap.dcMotor.get("rail");
        rail.setPower(0);
        rail.setDirection(DcMotor.Direction.REVERSE);
        rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ///Duck
        duck = hwMap.dcMotor.get("duck");
        duck.setPower(0);
        duck.setDirection(DcMotor.Direction.REVERSE);
        duck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ///Claw Servo
        claw = hwMap.servo.get("claw");
        claw.setPosition(claw_init);

        // imu initialization-this help the robot drive during autonomous
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";// see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrive and initialize the IMU. We expect the IMU to be attatched to an I2C import
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU"
        // and named IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // initialize TensorFlow
        initTfod(hwMap);

    }// end of init()

    // all other routines here

    public void driveMotors(double lp, double rp){
        leftfrontMotor.setPower(lp);
        rightfrontMotor.setPower(rp);
        leftbackMotor.setPower(lp);
        rightbackMotor.setPower(rp);
    }

    public void straightForTime(double pow, long ms) throws InterruptedException{
        driveMotors(pow,pow);
        sleep(ms);
        driveMotors(0,0);
    }

    public void straightIn(double pow, double dist_in) {
        // pow > 0 - move forward
        // pow < 0 - move backward

        long iniTime = System.currentTimeMillis();

        int l_cur_count = leftfrontMotor.getCurrentPosition();
        double tar_heading = get_heading();
        double l_target_count = l_cur_count;
        if(pow>0.0) l_target_count += dist_in * COUNT_PER_INCH;
        else l_target_count -= dist_in * COUNT_PER_INCH;


        driveMotors(0,0); ///Stop command
        leftfrontMotor.setTargetPosition((int)l_target_count);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        double l_pow = pow;

        double r_pow = pow;

        driveMotors(l_pow, r_pow);

        double cur_heading = get_heading();

        while (leftfrontMotor.isBusy() && ((System.currentTimeMillis()-iniTime)<10000)) { // timeout for 10 seconds

            cur_heading = get_heading();
            if (Math.abs(cur_heading-tar_heading)>0.5) { // trigger the imu direction correction
                if (cur_heading<tar_heading) { //
                    // slow down left motors
                    l_pow = pow * 0.9;
                } else {
                    // slow down right motors
                    r_pow = pow * 0.9;
                }
            }
            driveMotors(l_pow, r_pow);
        }
        driveMotors(0,0);
        leftfrontMotor.setPower(0);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    ///////////////////////////////////////////////////////////////
    public void claw_open(){
        claw.setPosition(claw_open);
    }

    public void claw_close() {  //82-99 give information to figure out servos
        claw.setPosition(claw_close);

    }

    public void claw_inc() {
        double cur_pos = claw.getPosition();
        double tar_pos = cur_pos + 0.01;
        if (tar_pos>1){
            tar_pos = 1;
        }
        claw.setPosition(tar_pos);
    }

    public void claw_dec() {
        double cur_pos = claw.getPosition();
        double tar_pos = cur_pos - 0.01;
        if (tar_pos<0){
            tar_pos = 0;
        }
        claw.setPosition(tar_pos);
    }
    ////////////////////////////////////////////////////////////

    ///rail
    public void rail_up(double power) {
        rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rail.setPower(power);
    }

    public void  rail_down(double power) {
        rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rail.setPower(-1*power);
    }

    public void rail_to_position(double power, int pos) {
        duck.setTargetPosition(pos);
        duck.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        duck.setPower(power);
    }


    public void rail_stop() {
        rail.setPower(0);
    }
    ///duck
    public void duck_up(double power) {
        duck.setPower(power);
    }

    public void  duck_down(double power) {
        duck.setPower(-1*power);
    }

    public void duck_stop() {
        duck.setPower(0);
    }
    ///More information for the IMU sensor

    public double get_heading() {
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    public void turn(double degree, double power) throws InterruptedException {
        // degree must be less than 100 degree, negative degree will do right turn, and positive will do left return
        double cur_heading = get_heading();
        double target_heading = cur_heading + degree;
        double lp = Math.abs(power);
        double rp = Math.abs(power);
        if (degree>0) { // left turn
            lp *= -1.0;
        } else { // right turn
            rp *= -1.0;
        }
        do {
            driveMotors(lp, rp);

        } while (Math.abs(get_heading()-target_heading)>2);

        driveMotors(0, 0);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        final String VUFORIA_KEY = "AYi9/Ez/////AAABmdzwEt9E9E3IkpfOhAio53AKkU2VM6zJkKPjbbRdvqHrsC/ZUqR7XC7uX/w5/qRn6J0/L40kRFzqSfZMfLtL4Z8W+tTzuTD9ilHRApdU63s4mbC7jgpXx3jUiDscBmadMLLqxf07qPIhpoawUT1KqAd7QRsjIDhRjCsxXpBcs5erRT7DwIv8NAJHkS5d9djM0Ap9z8+437ybIzL2x/UMleUz9B2oe7ifsXoC7s9RKj/kFlXKrK5NEc32SWIp2xpXioxM2Mj+z/VVXeleOBR/MMEYfTkxa73HeIEWhxUQdLK93KI3JBkETYzd9uBS8WzxYbYPa/zsftMuOtDeBgucaLm1Eq3YAWGzmQ7eVf/GTlck";
        final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
        String[] LABELS = {
                "Ball",
                "Cube",
                "Duck",
                "Marker"
        };

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.55f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0/9.0);
        }
    }
}// end of class FFHardware8132/// FFHardware8132(your Team Number 
