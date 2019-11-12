/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Platform Red", group="Linear Opmode")
//@Disabled
public class PlatformRed extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLeftDrive = null;
    private DcMotor FRightDrive = null;
    private DcMotor BLeftDrive = null;
    private DcMotor BRightDrive = null;
    private Servo LeftScissorLift = null;
    private Servo RightScissorLift = null;
    private Servo Dragger = null;
    private Gyroscope IMU = null;
    private Servo BlockGripper = null;
    private CRServo Extender = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Say", "Hello Driver");
        BLeftDrive = hardwareMap.get(DcMotor.class, "BLDr");
        BRightDrive = hardwareMap.get(DcMotor.class, "BRDr");
        FLeftDrive = hardwareMap.get(DcMotor.class, "FLDr");
        FRightDrive = hardwareMap.get(DcMotor.class, "FRDr");

        LeftScissorLift = hardwareMap.get(Servo.class, "LSc");
        RightScissorLift = hardwareMap.get(Servo.class, "RSc");
        BlockGripper = hardwareMap.get(Servo.class, "BG");
        Extender = hardwareMap.get(CRServo.class, "EX");
        Dragger = hardwareMap.get(Servo.class,"F");
        IMU = hardwareMap.get(Gyroscope.class,"imu");
        telemetry.addData("Status", "Initialized");

        BLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LeftScissorLift.setPosition(0.8);
        //RightScissorLift.setPosition(0.8);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        boolean reverse = false;
        boolean OnSite = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //encoders reset for each move
            Dragger.setPosition(.3);    // approx 60deg
            strafe(-500,0.75);
            drive(- 5300,0.75);     //forward (relative to hooks)
            Dragger.setPosition(1);     //down
            sleep(1000); //pause 1sec as servos set
            drive(5000,1); //backward (relative to hooks)
            turn(3000,1); //good luck with this
            Dragger.setPosition(0); //up
            drive(1000,1); //drive backward(rel hooks) to bridge
            sleep(1000);
            strafe(-7000,0.75);
            telemetry.addData("Done?","OnSite");
            telemetry.update();
            stop();
        }
    }
// Function to move the robot forward or back    (this may be turn)
    private void drive (int encoder,double p){
        boolean done = false;
        while(done != true && opModeIsActive()) {
            //CHANGE SIGN OF Y
            int FrontleftSet = encoder;
            int FrontrightSet = encoder;
            int BackleftSet = encoder;
            int BackrightSet = encoder;
            int FLPos = FLeftDrive.getCurrentPosition();
            int FRPos = FRightDrive.getCurrentPosition();
            int BLPos = BLeftDrive.getCurrentPosition();
            int BRPos = BRightDrive.getCurrentPosition();
            telemetry.addData("Drive: ", encoder);
            //telemetry.addData("Angle",IMU.getAngularVelocity());

            FLeftDrive.setTargetPosition(FrontleftSet);
            FLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRightDrive.setTargetPosition(FrontrightSet);
            FRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BLeftDrive.setTargetPosition(BackleftSet);
            BLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BRightDrive.setTargetPosition(BackrightSet);
            BRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int Range = 50;
            int FLDif = Math.abs(FrontleftSet - FLPos);
            int FRDif = Math.abs(FrontrightSet - FRPos);
            int BLDif = Math.abs(BackleftSet - BLPos);
            int BRDif = Math.abs(BackrightSet - BRPos);

            if ((FLDif <= Range) && (FRDif <= Range) && (BLDif <= Range) && (BRDif <= Range)) {
                FLeftDrive.setPower(0);
                FRightDrive.setPower(0);
                BLeftDrive.setPower(0);
                BRightDrive.setPower(0);
                BLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                done = true;
            } else {
                FLeftDrive.setPower(p);
                FRightDrive.setPower(p);
                BLeftDrive.setPower(p);
                BRightDrive.setPower(p);
                done = false;
            }
            telemetry.update();
        }
    }

    // Function to move the robot left or right
    private void strafe (int encoder,double p){
        boolean done = false;
        while(done != true && opModeIsActive()) {
            //CHANGE SIGN OF Y
            int FrontleftSet = -encoder;
            int FrontrightSet = encoder;
            int BackleftSet = encoder;
            int BackrightSet = -encoder;
            int FLPos = FLeftDrive.getCurrentPosition();
            int FRPos = FRightDrive.getCurrentPosition();
            int BLPos = BLeftDrive.getCurrentPosition();
            int BRPos = BRightDrive.getCurrentPosition();
            telemetry.addData("Strafe: ", encoder);
            //telemetry.addData("Angle",IMU.getAngularVelocity());

            FLeftDrive.setTargetPosition(FrontleftSet);
            FLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRightDrive.setTargetPosition(FrontrightSet);
            FRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BLeftDrive.setTargetPosition(BackleftSet);
            BLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BRightDrive.setTargetPosition(BackrightSet);
            BRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int Range = 50;
            int FLDif = Math.abs(FrontleftSet - FLPos);
            int FRDif = Math.abs(FrontrightSet - FRPos);
            int BLDif = Math.abs(BackleftSet - BLPos);
            int BRDif = Math.abs(BackrightSet - BRPos);

            if ((FLDif <= Range) && (FRDif <= Range) && (BLDif <= Range) && (BRDif <= Range)) {
                FLeftDrive.setPower(0);
                FRightDrive.setPower(0);
                BLeftDrive.setPower(0);
                BRightDrive.setPower(0);
                BLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                done = true;
            } else {
                FLeftDrive.setPower(p);
                FRightDrive.setPower(p);
                BLeftDrive.setPower(p);
                BRightDrive.setPower(p);
                done = false;
            }
            telemetry.update();
        }
    }

    // Function to TURN the robot left or right    (this may be turn)
    private void turn (int encoder,double p){
        boolean done = false;
        while(done != true && opModeIsActive()) {
            //CHANGE SIGN OF Y
            int FrontleftSet = encoder;
            int FrontrightSet = -encoder;
            int BackleftSet = encoder;
            int BackrightSet = -encoder;
            int FLPos = FLeftDrive.getCurrentPosition();
            int FRPos = FRightDrive.getCurrentPosition();
            int BLPos = BLeftDrive.getCurrentPosition();
            int BRPos = BRightDrive.getCurrentPosition();
            telemetry.addData("Turn: ", encoder);
            //telemetry.addData("Angle",IMU.getAngularVelocity());

            FLeftDrive.setTargetPosition(FrontleftSet);
            FLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRightDrive.setTargetPosition(FrontrightSet);
            FRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BLeftDrive.setTargetPosition(BackleftSet);
            BLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BRightDrive.setTargetPosition(BackrightSet);
            BRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            int Range = 50;
            int FLDif = Math.abs(FrontleftSet - FLPos);
            int FRDif = Math.abs(FrontrightSet - FRPos);
            int BLDif = Math.abs(BackleftSet - BLPos);
            int BRDif = Math.abs(BackrightSet - BRPos);

            if ((FLDif <= Range) && (FRDif <= Range) && (BLDif <= Range) && (BRDif <= Range)) {
                FLeftDrive.setPower(0);
                FRightDrive.setPower(0);
                BLeftDrive.setPower(0);
                BRightDrive.setPower(0);
                BLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                done = true;
            } else {
                FLeftDrive.setPower(p);
                FRightDrive.setPower(p);
                BLeftDrive.setPower(p);
                BRightDrive.setPower(p);
                done = false;
            }
            telemetry.update();
        }
    }


}
