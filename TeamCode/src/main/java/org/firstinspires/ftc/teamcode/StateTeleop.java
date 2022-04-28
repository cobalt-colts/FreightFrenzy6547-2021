package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name = "State Teleop")
public class StateTeleop extends LinearOpMode {
    //Calling Motors
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor ducks;
    private DcMotor intakeFront;
    private DcMotor linearSlide;
    private CRServo cServo;
    private Servo boxServo;
    private Servo udServo;
    private CRServo lrServo;
    private BNO055IMU imu;
    private ColorSensor color;

    //-------------------------
    int freight = 0; //Freight Location
    private double angleZeroValue = -3.1415 / 2.0;  // -pi/2
    double speedModifier = 0.8;
    double boxHome = 0.75;
    double boxDrive = 0.6;
    double boxDump = 0;
    double robotAngle = 0; //For Field Relative
    double lrPosition=0;
    double udPosition=0;
    boolean servoControl =false;
    boolean TSEmodeA = false;
    boolean TSEmodeB = false;
    //-------------------------
    Telemetry.Item Audrey;
    Telemetry.Item Joshua;
    Telemetry.Item LRServo;
    Telemetry.Item UDServo;
    Telemetry.Item currentFreight;
    //sounds
    @Override
    public void runOpMode(){
        //HwMappings
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        linearSlide=hardwareMap.get(DcMotor.class, "linearSlide");
        ducks = hardwareMap.get(DcMotor.class, "ducks");
        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        color = hardwareMap.get(ColorSensor.class, "Color");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        boxServo = hardwareMap.get(Servo.class, "boxServo");
        cServo = hardwareMap.get(CRServo.class, "cServo");
        udServo = hardwareMap.get(Servo.class, "udServo");
        lrServo = hardwareMap.get(CRServo.class, "lrServo");
        //----------------------------
        boxServo.setPosition(boxHome);
        initIMU();

        waitForStart();


        while (opModeIsActive()){
            //Telemetry and Running The Methods

            checkForMode();
            drivetrain();
            linearSlide();
            servoDump();
            intakeCheck();
            tseControl();
            spinDucks();
            freightChecker();
            updateTelemetry();



        }

    }



    public void checkForMode(){
        if(gamepad1.a){
            TSEmodeA = true;

        }else if(gamepad1.b){
            TSEmodeA = false;

        }

        if(gamepad2.a){
            TSEmodeB = true;

        }else if(gamepad2.b){
            TSEmodeB = false;


            udServo.setPosition(0);

        }
        if(gamepad1.a && gamepad1.b){
            //robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        }else{
            //robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
        }

    }

    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }


    public void drivetrain() {

        if (gamepad1.x) {
            angleZeroValue = this.getRawExternalHeading();

        }
        robotAngle = this.getRawExternalHeading() - angleZeroValue; //angle of robot

        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y); //get speed
        double LeftStickAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //get angle
        double rightX = -gamepad1.right_stick_x; //rotation
        rightX *= 0.8; //optionally reduce rotation value for better turning
        //linear the angle by the angle of the robot to make it field relative
        double leftFrontPower = speed * Math.cos(LeftStickAngle - robotAngle) + rightX;
        double rightFrontPower = speed * Math.sin(LeftStickAngle - robotAngle) - rightX;
        double leftBackPower = speed * Math.sin(LeftStickAngle - robotAngle) + rightX;
        double rightBackPower = speed * Math.cos(LeftStickAngle - robotAngle) - rightX;

        if(TSEmodeA){
            speedModifier =0.7;
        }else{
            speedModifier = .8 + (.8 * gamepad1.right_trigger) - (.4 * gamepad1.left_trigger);

        }

        leftFront.setPower(leftFrontPower * speedModifier);
        rightFront.setPower(rightFrontPower * speedModifier);
        leftBack.setPower(leftBackPower * speedModifier);
        rightBack.setPower(rightBackPower * speedModifier);

    }

    public void linearSlide() {
        if (gamepad2.x) {
            linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-80);
            linearSlide.setPower(-0.8);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (gamepad2.y) {
            //boxServo.setPosition(0.4);
            linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+80);
            linearSlide.setPower(0.8);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(gamepad2.dpad_down){
            servoControl = false;
            boxServo.setPosition(boxHome);
            linearSlide.setTargetPosition(0);
            linearSlide.setPower(-1);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Wait for box servo to go down


        }
        else if(gamepad2.dpad_up){
            boxServo.setPosition(boxDrive);

            linearSlide.setTargetPosition(2288);
            linearSlide.setPower(0.8);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoControl = true;



        } else if(gamepad2.dpad_left){
            boxServo.setPosition(boxDrive);

            linearSlide.setTargetPosition(1000);    // Was 1500
            linearSlide.setPower(0.8);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoControl = true;


        }


    }

    public void servoDump() {
        // //if(servoControl && gamepad2.right_stick_y < -0.1 && gamepad2.right_stick_y > -0.8){
        // boxServo.setPosition(gamepad2.right_stick_y+0.8);
        // }
        if(servoControl){
            if(gamepad2.right_stick_y < -0.1){
                boxServo.setPosition(gamepad2.right_stick_y);
            }else{
                boxServo.setPosition(boxDrive);


            }

        }else{
            boxServo.setPosition(boxHome);

        }


    }

    public void intakeCheck() {
        //This is the method to run the intake system, If you want intake to come in, press a,
        //or if you want to spit an object out, press b
        if(gamepad1.left_bumper || gamepad2.left_bumper){
            intakeFront.setPower(1);
        }else if(gamepad1.right_bumper || gamepad2.right_bumper){
            intakeFront.setPower(-1);
        }else{
            intakeFront.setPower(0);
        }

    }

    public void tseControl(){
        if(TSEmodeB) {


            if (gamepad2.left_trigger > 0) {
                cServo.setPower(1);
            } else if (gamepad2.right_trigger > 0) {
                cServo.setPower(-1);
            }else{
                cServo.setPower(0);
            }
            if(gamepad2.left_stick_y >0){
                udPosition -= 0.0025;

            }else if(gamepad2.left_stick_y <0){
                udPosition += 0.0025;
            }

            if(gamepad2.right_stick_x > 0){
                lrServo.setPower(0.125);
            }else if(gamepad2.right_stick_x < 0){
                lrServo.setPower(-0.125);
            }else{
                lrServo.setPower(0);
            }


            udServo.setPosition(0.5 + udPosition);

        }
    }

    public void spinDucks(){
        boolean isPressed1,isPressed2;

        if(TSEmodeA) {
            if (gamepad1.left_trigger > 0 && gamepad1.a) {
                ducks.setPower(1);
                isPressed1 = true;
            } else if (gamepad1.left_trigger > 0) {
                ducks.setPower(0.7);
                isPressed1 = true;
            }else{
                isPressed1 = false;
            }

            if (gamepad1.right_trigger > 0 && gamepad1.a) {
                ducks.setPower(-1);
                isPressed2 = true;
            } else if (gamepad1.right_trigger > 0) {
                ducks.setPower(-0.7);
                isPressed2 = true;
            }else{
                isPressed2 = false;
            }
            if(!isPressed1 && !isPressed2){
                ducks.setPower(0);
            }
        }
    }

    public void freightChecker(){
        double red = color.red();
        double green = color.green();
        double blue = color.blue();
        if(green>= 90){
            //ball
            freight = 1;

        }else if(Math.abs(red-blue) < 10){
            //freight
            freight = 2;
        }else{
            freight = 3;
        }

    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    public void updateTelemetry(){
        if (TSEmodeA) {
            Audrey = telemetry.addData("Audrey's Mode: ", "TSE");
        } else {
            Audrey = telemetry.addData("Audrey's Mode ", "Normal");

        }

        if (TSEmodeB) {
            Joshua = telemetry.addData("Joshua's Mode: ", "TSE");
        } else {
            Joshua = telemetry.addData("Joshua's Mode: ", "Normal");
        }

        if(freight == 1){
            currentFreight = telemetry.addData("Freight Detected: ", "ball");

        }else if(freight == 2){
            currentFreight = telemetry.addData("Freight Detected: ", "freight");

        }else{
            currentFreight = telemetry.addData("Freight Detected: ", "none");

        }
        UDServo = telemetry.addData("udServo Position ", udServo.getPosition());

        telemetry.update();
    }

}
//test hello