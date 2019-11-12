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

@Autonomous(name="Grab Platform", group="Linear Opmode")
//@Disabled
public class Grab_Platform extends LinearOpMode {

    // Declare OpMode members.
    //HardwarePushbot robot = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    public int LastX = 0;
    public int LastY = 0;
    public int LastR = 0;

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
    private void encodeReset(){
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
        LastR = 0;
        LastX = 0;
        LastY = 0;
    }
    private void moveTo(int x,int y,int r,double p){
        boolean done = false;
        while(done != true && opModeIsActive()) {
            int FrontleftSet = x + r - y;
            int FrontrightSet = x - r + y;
            int BackleftSet = x + r + y;
            int BackrightSet = x - r - y;
            int FLPos = FLeftDrive.getCurrentPosition();
            int FRPos = FRightDrive.getCurrentPosition();
            int BLPos = BLeftDrive.getCurrentPosition();
            int BRPos = BRightDrive.getCurrentPosition();
            telemetry.addData("FLPos: FLTarget", FLPos + " : " + FrontleftSet);
            telemetry.addData("FRPos: FRTarget", FRPos + " : " + FrontrightSet);
            telemetry.addData("BLPos: BLTarget", BLPos + " : " + BackleftSet);
            telemetry.addData("BRPos: BRTarget", BRPos + " : " + BackrightSet);
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
            Dragger.setPosition(0);
            moveTo(LastX - 5400,LastY + 0,LastR + 0,0.75);
            Dragger.setPosition(90);
            sleep(1000);
            moveTo(LastX + 1400,LastY + 0,LastR + 1700,1);
//            moveTo(LastX + 750,LastY + 0,LastR + 1500,1);
            Dragger.setPosition(0);
            sleep(1000);
            moveTo(LastX + 500,LastY + 11500,LastR + 0,1);
//            moveTo(LastX + 0,LastY + 10000,LastR + 0,1);
            sleep(3000);
            telemetry.addData("OnSite?","OnSite");
            telemetry.update();
            stop();
        }

    }

}
