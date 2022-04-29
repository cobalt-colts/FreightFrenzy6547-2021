package org.firstinspires.ftc.teamcode.drive.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.auton.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;

@Autonomous(name = "RR Auton Red Warehouse", group = "World")

public class RRopenCVRedWarehouse extends LinearOpMode {
    //Various Variables ;)

    enum Cases{
        START,
        FIRST_CYCLE,
        FIRST_INTAKE,
        SECOND_CYCLE,
        SECOND_INTAKE,
        THIRD_CYCLE,
        END

    }

    Cases cases = Cases.START;
    double linearSlideTarget;
    double targetForward;
    double boxHome = .75;
    double boxDrive = 0.5;
    double boxDump = 0;
    double linearSlideHigh = -2170;
    double linearSlideLow = 300;
    int autonLocation;
    int redThresh = 40; int greenThresh = 40; int blueThresh = 40;

    //Pipeline Object
    OpenCVPipeline testOpenCV = new OpenCVPipeline(telemetry);

    Pose2d i1Pose;
    Pose2d startPose = new Pose2d(11,-62,Math.toRadians(270));
    //change startingpos angle to 3PI/2

    @Override
    public void runOpMode() throws InterruptedException{
        //HardwareMapping and Roadrunner
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //Setting up OpenCV
        //----------------
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        drive.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        drive.webcam.setPipeline(testOpenCV);
        drive.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                drive.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        //----------------
        drive.linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.setPoseEstimate(startPose);
        linearSlideStart(drive);


        TrajectorySequence ts1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> RTPlinearSlideUp(drive,linearSlideTarget))
                .lineToSplineHeading(new Pose2d(-5, -36, Math.toRadians(300)))
                //.strafeTo(new Vector2d(-12,-34))
                //.strafeTo(new Vector2d(-9,42)) //goes to alliance hub while moving linearSlideUp
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> boxServoDump(drive)) //score the freight
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> boxServoHome(drive)) //return box servo
                .waitSeconds(1)
                //FIRST LOOP

                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> RTPlinearSlideDown(drive, 0)) //return linear slide
                .lineToSplineHeading(new Pose2d(5, -72, Math.toRadians(0))) //get ready to go to warehouse
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.intake.setPower(0.7)) //turn on intake motors
                .strafeTo(new Vector2d(48,  -72)) //go to warehouse
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.intake.setPower(0))
                .lineToSplineHeading(new Pose2d(48,-54,Math.toRadians(90)))

                .build();

        TrajectoryVelocityConstraint tvc= new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(50),
                new AngularVelocityConstraint(1)
        ));
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.intake.setPower(-0.7)) //outtake
                //.strafeTo(new Vector2d(50,65)) //making sure we don't go over the barrier
                .strafeTo(new Vector2d(10,-73))
                .strafeTo(new Vector2d(6,-72)) //get ready to score again
                .strafeLeft(3)

                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> drive.intake.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> RTPlinearSlideUp(drive,linearSlideHigh))
                .lineToSplineHeading(new Pose2d(-2, -40, Math.toRadians(300)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> boxServoDump(drive)) //score the freight
                .UNSTABLE_addTemporalMarkerOffset(1.9, () -> boxServoHome(drive)) //return box servo
                .waitSeconds(2)
                //THIRD LOOP

                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> RTPlinearSlideDown(drive, -330)) //return linear slide
                .lineToSplineHeading(new Pose2d(9, -72, Math.toRadians(0))) //get ready to go to warehouse
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.intake.setPower(0.7)) //turn on intake motors
                .strafeTo(new Vector2d(50,  -72)) //go to warehouse
                //END OF SECOND STATEMENT
                .waitSeconds(2)
                .build();

        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(ts2.end())
                //START OF THIRD STATEMENT
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.intake.setPower(-0.7)) //outtake
                //.strafeTo(new Vector2d(50,65)) //making sure we don't go over the barrier
                .strafeTo(new Vector2d(9,-72)) //get ready to score again
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> drive.intake.setPower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> RTPlinearSlideUp(drive,linearSlideHigh))
                .lineToSplineHeading(new Pose2d(-2, -40, Math.toRadians(300)))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> boxServoDump(drive)) //score the freight
                .UNSTABLE_addTemporalMarkerOffset(1.9, () -> boxServoHome(drive)) //return box servo
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> RTPlinearSlideDown(drive, -300)) //return linear slide
                .lineToSplineHeading(new Pose2d(9, -72, Math.toRadians(0))) //get ready to go to warehouse
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.intake.setPower(0.7)) //turn on intake motors
                .strafeTo(new Vector2d(50,  -70)) //go to warehouse
                .build();


        TrajectorySequence i1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(1)
                .build();

        while(!opModeIsActive() && !isStopRequested()){
            //during the init phase we frequently update the location
            autonLocation = testOpenCV.getDuckLocation();
            //telemetry.addData("Area: ", autonCV.getAreaUpper());
            telemetry.addData("Location: ", testOpenCV.getDuckLocation());
            telemetry.update();

        }
        drive.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        waitForStart();
        Thread stopCamera = new Thread(() -> drive.webcam.stopStreaming());
        stopCamera.start();
        i1Pose = ts1.end();

        drive.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        switch(autonLocation){
            case 1:
                linearSlideTarget = -700;
                break;
            case 2:
                linearSlideTarget = -1300; //@TODO Check linslide values
                break;
            case 3:
                linearSlideTarget = -1600;
                break;
        }
        //------------
        boolean i1First = true;
        double red,green,blue;
        linearSlideStart(drive);
        while(!isStopRequested() && cases != Cases.END){
            switch(cases){
                case START:
                    cases = Cases.FIRST_CYCLE;
                    break;
                case FIRST_CYCLE:
                    drive.followTrajectorySequence(ts1);
                    cases = Cases.END; //@TODO more freight when time
                    break;
                case FIRST_INTAKE:

                    drive.followTrajectorySequence(i1);

                    red = drive.color.red();
                    green = drive.color.green();
                    blue = drive.color.blue();

                    if(green<90&& Math.abs(red-blue) >= 10){
                        cases = Cases.SECOND_CYCLE;
                    }
                    break;
                case SECOND_CYCLE:
                    drive.followTrajectorySequence(ts2);
                    cases = Cases.THIRD_CYCLE;
                    break;
                case THIRD_CYCLE:
                    drive.followTrajectorySequence(ts3);
                    cases = Cases.END;
                    break;

            }
            drive.update();
            drive.updatePoseEstimate();

            telemetry.addData("Linear slide pose: ", drive.linearSlide.getCurrentPosition());
            telemetry.update();
        }
        linearSlideDown(drive);
//        PoseStorage.currentPose = drive.getPoseEstimate(); //transferring this to WorldTeleopWithRoadrunner

    }
    public void RTPlinearSlideUp(SampleMecanumDrive drive, double linearSlideTarget){
        drive.boxServo.setPosition(0.6);
        drive.linearSlide.setTargetPosition((int) linearSlideTarget);
        drive.linearSlide.setPower(-1);
        drive.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void RTPlinearSlideDown(SampleMecanumDrive drive, double linearSlideTarget){

        drive.linearSlide.setTargetPosition((int) linearSlideTarget);
        drive.linearSlide.setPower(1);
        drive.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.boxServo.setPosition(boxHome);

    }

    public void linearSlideStart(SampleMecanumDrive drive){
        drive.linearSlide.setPower(-.05);
        sleep(2000);
        drive.linearSlide.setPower(0);
        drive.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void linearSlideUp(SampleMecanumDrive drive, double linearSlideTarget){
        double startTime = System.currentTimeMillis();
        telemetry.addData("linearSlideUp", 0);
        telemetry.update();
        // Run it up for 3 seconds. After 3 seconds, give up.
        while(opModeIsActive() && drive.linearSlide.getCurrentPosition() > linearSlideTarget && System.currentTimeMillis() < startTime+3000){
            drive.linearSlide.setPower(-0.8);
            telemetry.addData("linearSlide Position: ", drive.linearSlide.getCurrentPosition());
            telemetry.addData("linearSlide target: ", linearSlideTarget);
            telemetry.update();

        }
        drive.linearSlide.setPower(0);
    }

    public void linearSlideDown(SampleMecanumDrive drive){
        telemetry.addData("linearSlideDown", 0);
        telemetry.update();
        while(opModeIsActive() && drive.linearSlide.getCurrentPosition() < linearSlideLow){
            drive.linearSlide.setPower(1);
            telemetry.addData("linearSlide Position: ", drive.linearSlide.getCurrentPosition());
            telemetry.addData("linearSlide target: ", linearSlideLow);
            telemetry.update();
        }
        drive.linearSlide.setPower(0);

    }

    public void boxServoDump(SampleMecanumDrive drive) {
        drive.boxServo.setPosition(boxDump);
    }
    public void boxServoHome(SampleMecanumDrive drive) {
        drive.boxServo.setPosition(boxHome);
    }


}