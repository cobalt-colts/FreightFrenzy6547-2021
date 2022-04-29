package org.firstinspires.ftc.teamcode.drive.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.auton.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Vector;

@Autonomous(name = "RR Auton Red Duck", group = "World")

public class RRopenCVRedDuck extends LinearOpMode {
    //Various Variables ;)

    enum Cases{
        START,
        GO_CAROUSEL,
        SPIN_DUCKS,
        SCORE_PRELOAD,
        PARK,
        END

    }

    Cases cases = Cases.START;
    double linearSlideTarget;
    double targetForward;
    double boxHome = .75;
    double boxDrive = 0.5;
    double boxDump = 0;
    double linearSlideHigh = -2170;
    double linearSlideLow = -0;
    int autonLocation;
    int redThresh = 40; int greenThresh = 40; int blueThresh = 40;

    //Pipeline Object
    OpenCVPipeline testOpenCV = new OpenCVPipeline(telemetry);

    Pose2d i1Pose;
    Pose2d startPose = new Pose2d(-35,-62,Math.toRadians(270));
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


//        Trajectory GOTO_CAROUSEL = drive.trajectoryBuilder(startPose)
//                .strafeTo(new Vector2d(-62,-54))
//
//                .build();

        TrajectorySequence SCORE_PRELOAD = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-62,-54))

                .lineToLinearHeading(new Pose2d(-63,-30,Math.toRadians(269)))
                .lineToLinearHeading(new Pose2d(-52,-24, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> RTPlinearSlideUp(drive,linearSlideTarget))
                .strafeTo(new Vector2d(-26,-24))
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> boxServoDump(drive)) //score the freight
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> drive.intake.setPower(0.8))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> boxServoHome(drive)) //return box servo
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> RTPlinearSlideDown(drive, -330)) //return linear slide
                .lineToSplineHeading(new Pose2d(-63,-24,Math.toRadians(270)))
//                .strafeTo(new Vector2d(-63, -35))
//                .strafeTo(new Vector2d(-56
//                        ,-60))
//
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(-40,-59))
//                .strafeTo(new Vector2d(-50, -63))
//                .UNSTABLE_addTemporalMarkerOffset(0.3,() -> RTPlinearSlideUp(drive,-2170))
//                .waitSeconds(0.4)
//                .strafeTo(new Vector2d(-17,-40))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.3,()->boxServoDump(drive))
//                .waitSeconds(2)
//                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> boxServoHome(drive))
//                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> RTPlinearSlideDown(drive,-300))
//                .waitSeconds(1)
                .build();
        TrajectorySequence SPIN_DUCK = drive.trajectorySequenceBuilder(SCORE_PRELOAD.end())
                .lineToLinearHeading(new Pose2d(-62,-59, Math.toRadians(255)))

                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.duckSpinner.setPower(-.3) )
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.duckSpinner.setPower(-1) )
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> drive.duckSpinner.setPower(0) )
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->drive.intake.setPower(1))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> drive.intake.setPower(0))
                //.strafeLeft(7)
                .waitSeconds(1.0)

                .strafeTo(new Vector2d(-62,-54))

//                .lineToLinearHeading(new Pose2d(-63,-30,Math.toRadians(269)))
//                .lineToLinearHeading(new Pose2d(-52,-24, Math.toRadians(180)))
//                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> RTPlinearSlideUp(drive,linearSlideTarget))
//                .strafeTo(new Vector2d(-26,-24))
//                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> boxServoDump(drive)) //score the freight
//                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> drive.intake.setPower(0.8))
//                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> boxServoHome(drive)) //return box servo
//                .waitSeconds(1)
//                .UNSTABLE_addTemporalMarkerOffset(0.1,() -> RTPlinearSlideDown(drive, -340)) //return linear slide
//                .lineToSplineHeading(new Pose2d(-63,-24,Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-63.1,-38,Math.toRadians(90)))



                .build();
        while(!opModeIsActive() && !isStopRequested()){
            //during the init phase we frequently update the location
            autonLocation = testOpenCV.getDuckLocation();
            //telemetry.addData("Area: ", autonCV.getAreaUpper());
            telemetry.addData("Location: ", testOpenCV.getDuckLocation());
            telemetry.update();

        }
        drive.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);

        waitForStart();

        if(isStopRequested()) return;

        drive.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        switch(autonLocation){
            case 1:
                linearSlideTarget = -700;
                break;
            case 2:
                linearSlideTarget = -1100;
                break;
            case 3:
                linearSlideTarget = -1970;
                break;
        }
        //------------
        linearSlideStart(drive);

        cases = Cases.SCORE_PRELOAD;

        while(opModeIsActive()&&!isStopRequested()){
            switch(cases){

                case SCORE_PRELOAD:
                    drive.followTrajectorySequence(SCORE_PRELOAD);
                    cases = Cases.SPIN_DUCKS;
                    break;
                case SPIN_DUCKS:
                    drive.followTrajectorySequence(SPIN_DUCK);
                    cases = Cases.END;
                    break;

                case END:
                    linearSlideDown(drive);
                    break;
            }
            drive.updatePoseEstimate();
            drive.update();
        }


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
        drive.linearSlide.setPower(.05);
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