package org.firstinspires.ftc.teamcode.drive.teleop;

import static java.lang.Math.abs;

import com.qualcomm.ftccommon.SoundPlayer;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "World Teleop")
public class WorldTeleop extends LinearOpMode {
    //Calling Motors
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor ducks;
    private DcMotor intakeFront;
    private DcMotor linearSlide;
    private Servo boxServo;
    private Servo eyes;
    private CRServo cServo;
    private CRServo udServo;
    private CRServo lrServo;
    private BNO055IMU imu;
    private ColorSensor color;
    private RevBlinkinLedDriver lights;
    private DistanceSensor distanceA;
    private DistanceSensor distanceB;

    private ElapsedTime    servoOut = new ElapsedTime();
    private ElapsedTime    servoIn = new ElapsedTime();

    //-------------------------
    int freight = 0; //Freight Location
    private double angleZeroValue = -3.1415 / 2.0;  // -pi/2
    double speedModifier = 0.8;
    double boxHome = 0.75;
    double boxDrive = 0.5;
    double boxDump = 0;
    double robotAngle = 0; //For Field Relative
    double lrPosition=0;
    double udPosition=0;
    double startExtend = 0;
    double startRetract = 0;
    double totalExtend = 0;
    double movingExtend = 0;
    double movingRetract = 0;

    double udModifier = 0;
    double lrModifier = 0;
    boolean servoControl =false;

    boolean automatedIntake = false;
    boolean prevB = false;
    boolean currB = false;


    boolean TSEmodeB = false;
    boolean prevA = false;
    boolean currA = false;


    public boolean audreyRumble = true;
    public boolean joshuaRumble = true;
    //-------------------------

    Telemetry.Item Joshua;
    Telemetry.Item LRServo;
    Telemetry.Item UDServo;
    Telemetry.Item currentFreight;

    //Timer
    public ElapsedTime endgameWarning = new ElapsedTime();

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
        udServo = hardwareMap.get(CRServo.class, "udServo");
        lrServo = hardwareMap.get(CRServo.class, "lrServo");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        distanceA = hardwareMap.get(DistanceSensor.class,"distanceA");
        distanceB = hardwareMap.get(DistanceSensor.class,"distanceB");
        eyes = hardwareMap.get(Servo.class, "eyes");
        //----------------------------
        boxServo.setPosition(boxHome);
        ducks.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initIMU();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
        boolean startTime = false;
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
            rumbleOverride();
            eyes();
            if(!TSEmodeB){
                freightChecker();
            }
            if(!startTime){
                endgameWarning.reset();
                startTime = true;

            }

            telemetry.addData("Time Running: ", endgameWarning.seconds());
            updateTelemetry();
            endgameTimeWarning();


        }

    }

    public void endgameTimeWarning(){
        if(endgameWarning.seconds() >= 83 && endgameWarning.seconds() <= 84){
            gamepad1.rumble(0.7,0.7,100);
            gamepad2.rumble(0.7,0.7,100);
        }else if(endgameWarning.seconds() >= 84 && endgameWarning.seconds() <= 85){
            gamepad1.rumble(0.7,0.7,100);
            gamepad2.rumble(0.7,0.7,100);
        }else if(endgameWarning.seconds() >= 85 && endgameWarning.seconds() <= 86){
            gamepad1.rumble(0.7,0.7,100);
            gamepad2.rumble(0.7,0.7,100);
        }

        if(endgameWarning.seconds() == 115){
            gamepad1.rumble(0.7,0.7,300);
            gamepad2.rumble(0.7,0.7,300);
        }
    }



    public void checkForMode(){

        currA = gamepad2.a;

        if(currA && !prevA){
            TSEmodeB = !TSEmodeB;

        }

        prevA =gamepad2.a;


        if(TSEmodeB){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        }else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
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



        speedModifier = .8 + (.8 * gamepad1.right_trigger);

        if(gamepad1.left_trigger > .3) {
            speedModifier = 0;
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        leftFront.setPower(leftFrontPower * speedModifier);
        rightFront.setPower(rightFrontPower * speedModifier);
        leftBack.setPower(leftBackPower * speedModifier);
        rightBack.setPower(rightBackPower * speedModifier);

    }

    public void linearSlide() { //@TODO CHECK LINEAR SLIDE VALUES
        if (gamepad2.x) {
            linearSlide.setTargetPosition(linearSlide.getCurrentPosition()-400);
            linearSlide.setPower(-1);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (gamepad2.y) {
            //boxServo.setPosition(0.4);
            linearSlide.setTargetPosition(linearSlide.getCurrentPosition()+180);
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

            linearSlide.setTargetPosition(1600);
            linearSlide.setPower(0.8);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoControl = true;



        } else if(gamepad2.dpad_left){
            boxServo.setPosition(boxDrive);

            linearSlide.setTargetPosition(1200);    // Was 1500
            linearSlide.setPower(0.8);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            servoControl = true;


        }else if(gamepad2.dpad_right){
            boxServo.setPosition(boxDrive);

            linearSlide.setTargetPosition(800);    // Was 1500
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

    public void eyes(){
        eyes.setPosition(gamepad1.right_stick_x);
    }


    public void tseControl(){
        if(TSEmodeB) {
            telemetry.addData("totalExtend: ", totalExtend);
//smh shishir comment the code lol
            if (gamepad2.left_trigger > 0.1) {
                startRetract = servoIn.seconds(); //when left trigger is pressed the tape goes back in
                cServo.setPower(-gamepad2.left_trigger);
            } else if (gamepad2.right_trigger > 0.1) {
                startExtend = servoOut.seconds(); //when right trigger is pressed the tape goes forward
                cServo.setPower(gamepad2.right_trigger);
            }else{
                movingRetract = servoIn.seconds() - startRetract; //nothing is pressed,
                movingExtend = servoOut.seconds() - startExtend;
                cServo.setPower(0);
                //udModifier = ;
            }
            totalExtend = Math.min(0, movingExtend - movingRetract);
            udModifier = Math.max(0.2,.5 - (abs(totalExtend-20))/120);
            lrModifier = Math.max(0.2,.5 - (abs(totalExtend-20))/120);
            udServo.setPower(udModifier * gamepad2.left_stick_y);
            lrServo.setPower(lrModifier * -gamepad2.right_stick_x);
//0.8 0.1125
//0.6 0.1
//0.4 0.05
//0.2 0.025
//0 0
//            if(gamepad2.right_stick_x < 0){
//                if(gamepad2.right_stick_x > -0.2){
//                    lrServo.setPower(0.025);
//                }else if(gamepad2.right_stick_x > -0.4){
//                    lrServo.setPower(0.05);
//                }else if(gamepad2.right_stick_x > -0.6){
//                    lrServo.setPower(0.1);
//                }else if(gamepad2.right_stick_x > -0.8){
//                    lrServo.setPower(0.1125);
//                }else{
//                    lrServo.setPower(0.125);
//                }
//            }else if(gamepad2.right_stick_x > 0){
//                if(gamepad2.right_stick_x < 0.2){
//                    lrServo.setPower(-0.025);
//                }else if(gamepad2.right_stick_x < 0.4){
//                    lrServo.setPower(-0.05);
//                }else if(gamepad2.right_stick_x < 0.6){
//                    lrServo.setPower(-0.1);
//                }else if(gamepad2.right_stick_x < 0.8){
//                    lrServo.setPower(-0.1125);
//                }else{
//                    lrServo.setPower(-0.125);
//                }
//            }else{
//                lrServo.setPower(0);
//            }




        }
    }

    public void spinDucks(){
        boolean isPressed1,isPressed2;


        if (gamepad1.dpad_up && gamepad1.a) {
            ducks.setPower(1);
            isPressed1 = true;
        } else if (gamepad1.dpad_up) {
            ducks.setPower(0.7);
            isPressed1 = true;
        }else{
            isPressed1 = false;
        }

        if (gamepad1.dpad_down && gamepad1.a) {
            ducks.setPower(-1);
            isPressed2 = true;
        } else if (gamepad1.dpad_down) {
            ducks.setPower(-0.7);
            isPressed2 = true;
        }else{
            isPressed2 = false;
        }
        if(!isPressed1 && !isPressed2){
            ducks.setPower(0);
        }

    }


    public void intakeCheck() {
        double speed;
        currB = gamepad2.b;

        if(currB && !prevB){
            automatedIntake= !automatedIntake;

        }
        //This is the method to run the intake system, If you want intake to come in, press a,
        //or if you want to spit an object out, press b
        if(!automatedIntake) {
            speed = 0.6;
        }else{
            speed = 0.8;
        }

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            intakeFront.setPower(speed); //@TODO check servo speed
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intakeFront.setPower(-speed);
        } else {
            intakeFront.setPower(0);
        }
        prevB = gamepad2.b;

    }
    public void freightChecker(){
        double red = color.red();
        double green = color.green();
        double blue = color.blue();
        ElapsedTime outtakeTimer = new ElapsedTime();
//        if(green>= 100){
//            //ball
//            freight = 1;
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//            if(audreyRumble){gamepad1.rumble(0.7,0.7,300);}
//            if(joshuaRumble){gamepad2.rumble(0.7,0.7,300);}
//            if(automatedIntake){intakeFront.setPower(-0.6);}
//        }else if(abs(red+green) < 150){
//            //freight
//            freight = 2;
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//            if(audreyRumble){gamepad1.rumble(0.7,0.7,300);}
//            if(joshuaRumble){gamepad2.rumble(0.7,0.7,300);}
//            if(automatedIntake){intakeFront.setPower(-0.6);}
//
//        }else{
//            freight = 3;
//            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
//            gamepad1.stopRumble();
//            gamepad2.stopRumble();
//            if(automatedIntake){intakeFront.setPower(0.6);}
//        }
//
        if(red <= 55){
            freight = 3;
        }else{
            if(audreyRumble){gamepad1.rumble(0.7,0.7,300);}
            if(joshuaRumble){gamepad2.rumble(0.7,0.7,300);}
            freight = 1;
        }
        //51 80 65 is none
        //60 96 77is ball
        //58 88 66 is freight
    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    public void rumbleOverride(){
        if(gamepad1.x){
            audreyRumble = true;
            joshuaRumble = true;
        }else if(gamepad1.y){
            audreyRumble = false;
            joshuaRumble = false;
        }
        //if(gamepad2.x){
        //    joshuaRumble = true;
        //}else if(gamepad2.y){
        //    joshuaRumble = false;
        //}
    }

    public void updateTelemetry(){

        if(automatedIntake){
            telemetry.addData("Automated Intake On.", " Press B to toggle");
        }else{
            telemetry.addData("Automated Intake Off.", " Press B to toggle");
        }
        if (TSEmodeB) {
            Joshua = telemetry.addData("Joshua's Mode: ", "TSE");
        } else {
            Joshua = telemetry.addData("Joshua's Mode: ", "Normal");
        }

        if(freight == 1){
            currentFreight = telemetry.addData("Freight Detected: ", "freight");

        }else if(freight == 2){
            currentFreight = telemetry.addData("Freight Detected: ", "freight");

        }else{
            currentFreight = telemetry.addData("Freight Detected: ", "none");

        }
        telemetry.addData("RGB Val: " + color.red() + " " + color.green() + " " + color.blue(),"");
        telemetry.addData("Audrey Rumble (Y to disable): ", audreyRumble);
        telemetry.addData("Joshua Rumble (Y to disable): ", joshuaRumble);

        telemetry.addData("Up Down Servo:", udServo.getPower());
        telemetry.addData("Left Right Servo:", lrServo.getPower());
        telemetry.update();
    }

}