package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vision.MasterVision;
import org.firstinspires.ftc.teamcode.vision.SampleRandomizedPositions;

@Autonomous(name = "RedDepot", group = "auto")



public class RedDepot extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;
    public DcMotor landingSlide;
    public CRServo depotServo;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "Acw8Gxf/////AAABmZK7XZVv1kl7jD8H8KMhrZdShtseDx4Gw0BrXsz+zv1HOnK7rlVa8nPFXlOfuQGckPwcq72CihPxljGaI1NhbYPqWa/JKc+mRhNVdPO9ucPjZ6P20cZeIGpFrDKmctqdxNkmf6sAZGqJhMgFGZmLHl7v5ZAfbCmV/pzuBmqW0EojGw6FarJ0IleIWgLfjs6MW9guYQzMbXCtMoZ/eDWHrhwLH22c2SSWlDg8pAyaytcd6/Y7JPwwmtqdnC7JpEMOKyDISIaRkyRePU/O+vBrxdy3i0vHNlObaipL7zCcSk/kvMqu1cuJ7Wj/anI7RepfBMLeEsHFqH26mPxnkVhzin/TNXtw/qRQO1xJNN93DSso";
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "ld");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rd");
        landingSlide = hardwareMap.get(DcMotor.class, "latch");
        depotServo  = hardwareMap.get(CRServo.class, "depotServo");


        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_LEFT);
        vision.init();

        runtime.reset();
        landingSlide.setPower(-0.6);


        // enables the camera overlay. this will take a couple of seconds


        waitForStart();

        boolean slideDone = false;
        landingSlide.setPower(0);
        sleep(2000);
        vision.enable();// enables the tracking algorithms. this might also take a little time
        //wait(1000);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderDrive(0.5, 130, 130, 130, 130, 3.0, " disengage from lander");
        encoderDrive(0.5, -260, 260, -260, 260, 3.0, " strafe left to identify minerals");

        sleep(2500);






        //encoderDrive(0.2,200, 200, 200, 200, 5.0);


        vision.disable();


        goldPosition = vision.getTfLite().getLastKnownSampleOrder();


        int done = 0;
        while(opModeIsActive() && done < 1){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    encoderDrive(0.4, -450, -450, -450, -450, 4.0, "align with gold ");
                    encoderDrive(0.4, -500, 500, -500, 500, 4.0, " hit gold");
                    encoderDrive(0.4, -100, -100, -100, -100, 4.0, "move backwards to prepare for drop ");

                    encoderDrive(0.4, 150, -150, -150, 150, 4.0, " turn to parallel with depot");
                    encoderDrive(0.4, -250, 250, 250, -250, 4.0, "strafe into depot & drop ");


                    depotServo.setPower(-1);
                    sleep(1500);

                    encoderDrive(0.4, 1400, 1400, 1400, 1400, 4.0, "drive into crater ");

                    done = done +1;

                    break;
                case CENTER:
                    encoderDrive(0.4, -320, -320, -320, -320, 4.0, "align with gold ");
                    encoderDrive(0.4, -500, 500, -500, 500, 4.0, " hit gold ");

                    encoderDrive(0.4, 150, -150, -150, 150, 4.0, " turn to face crater");

                    encoderDrive(0.4, -200, 200, -200, 200, 4.0, " strafe to hit wall");

                    encoderDrive(0.4, -200, -200, -200, -200, 4.0, "move into depot ");





                    depotServo.setPower(-1);
                    sleep(1500);

                    encoderDrive(0.4, 1300, 1300, 1300, 1300, 4.0, "drive into crater");

                    done = done +1;


                    telemetry.addLine("going straight");
                    break;
                case RIGHT:
                    encoderDrive(0.4, 80, 80, 80, 80, 4.0, " align with gold ");
                    encoderDrive(0.4, -500, 500, -500, 500, 4.0, "hit gold ");
                    encoderDrive(0.4, 150, -150, -150, 150, 4.0, "turn to face crater ");
                    encoderDrive(0.4, -400, -400, -400, -400, 4.0, "drive into depot");




                    depotServo.setPower(-1);
                    sleep(1500);
                    encoderDrive(0.4, 1300, 1300, 1300, 1300, 4.0, " ");
                    telemetry.addLine("going to the right");
                    done = done +1;
                    break;
                case UNKNOWN:
                    encoderDrive(0.4, -500, 500, -500, 500, 4.0, "hit gold ");
                    encoderDrive(0.4, 150, -150, -150, 150, 4.0, "turn to face crater ");
                    encoderDrive(0.4, -400, -400, -400, -400, 4.0, "drive into depot");




                    depotServo.setPower(-1);
                    sleep(1500);
                    encoderDrive(0.4, 1300, 1300, 1300, 1300, 4.0, " ");
                    telemetry.addLine("staying put");
                    done = done +1;
                    break;
            }

            telemetry.update();
        }

        vision.shutdown();
    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double rightBackInches, double leftBackInches,
                             double timeoutS, String comment) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (leftFrontInches);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (rightFrontInches );
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (leftBackInches );
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (rightBackInches);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Comment: ", comment);
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    } }
