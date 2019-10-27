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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleopv1.1", group="Iterative Opmode")
//@Disabled
public class Our_New_TeamCode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;
    private DcMotor FleftDrive = null;
    private DcMotor FrightDrive = null;
    private Servo Leftscissorlift = null;
    private Servo Rightscissorlift = null;
    private Servo BlockGripper = null;
    private CRServo Extender = null;
    private Servo Foundation = null;
    double ScissorPostion = 0.8;
    double GripperPosition = 0.2;
    double FoundationPosition= 0.2;
    boolean AState=false;
    boolean YState=false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        BleftDrive  = hardwareMap.get(DcMotor.class, "BLDr");
        BrightDrive = hardwareMap.get(DcMotor.class, "BRDr");
        FleftDrive = hardwareMap.get(DcMotor.class, "FLDr");
        FrightDrive = hardwareMap.get(DcMotor.class, "FRDr");
        Leftscissorlift = hardwareMap.get(Servo.class, "LSc");
        Rightscissorlift = hardwareMap.get(Servo.class, "RSc");
        BlockGripper = hardwareMap.get(Servo.class, "BG");
        Extender = hardwareMap.get(CRServo.class, "EX");
        Foundation = hardwareMap.get(Servo.class, "F");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
        // FleftDrive.setDirection(DcMotor.Direction.FORWARD);
       FrightDrive.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /****************************************Mecanum POV Drive ******************************************/
        // Setup a variable for each drive wheel to save power level for telemetry
        double FrontleftPower;
        double BackleftPower;
        double FrontrightPower;
        double BackrightPower;

        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = -gamepad1.left_stick_x;

        // POV Mode uses left stick for point of view drive, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        FrontleftPower = Range.clip(Speed + Turn - Strafe, -1.0, 1.0);
        FrontrightPower = Range.clip(Speed - Turn + Strafe, -1.0, 1.0);
        BackleftPower = Range.clip(Speed + Turn + Strafe, -1.0, 1.0);
        BackrightPower = Range.clip(Speed - Turn - Strafe, -1.0, 1.0);

        // Send calculated power to wheels
        FleftDrive.setPower(FrontleftPower);
        FrightDrive.setPower(FrontrightPower);
        BleftDrive.setPower(BackleftPower);
        BrightDrive.setPower(BackrightPower);

        /*******************************************Scissor Lift Code**********************************************/
        //Buttons
        if (gamepad2.y != YState) {
            YState = gamepad2.y;
            if (gamepad2.y) {
                ScissorPostion = ScissorPostion + 0.1;
            }
        } else if (gamepad2.a != AState) {
            AState = gamepad2.a;
            if (gamepad2.a) {
                ScissorPostion = ScissorPostion - 0.1;
            }
        } else if (gamepad2.b) {
            ScissorPostion = 0.5;
        }

        //Joystick
        else if (gamepad2.right_stick_y > .5) {
        telemetry.addData("subjoy","");
            ScissorPostion = ScissorPostion - .01;
           while (runtime.milliseconds() < 100) { };
            runtime.reset();
        }
        else if (gamepad2.right_stick_y < -.5)
        {
            telemetry.addData("addJOY","");
            ScissorPostion = ScissorPostion + .01;
            while (runtime.milliseconds() < 100) { };
            runtime.reset();

            }

        else {ScissorPostion= ScissorPostion;}
        Leftscissorlift.setPosition(ScissorPostion);
        Rightscissorlift.setPosition(ScissorPostion);


        /*******************************************Gripper Code**********************************************/
        if (gamepad2.left_bumper)
            GripperPosition=.2;
        else if (gamepad2.right_bumper)
            GripperPosition=.8;
        else
            GripperPosition=GripperPosition;

        BlockGripper.setPosition(GripperPosition);

        telemetry.addData("LServo", Leftscissorlift.getPosition());
        telemetry.addData("RServo", Rightscissorlift.getPosition());

/*************************************************Extender Code************************************************/

        Extender.setPower(gamepad2.left_stick_y);

/*****************************************************Foundation****************************************/
        if (gamepad1.right_bumper)
            FoundationPosition=.2;
        else if (gamepad1.left_bumper)
            FoundationPosition=.8;
        else
            FoundationPosition=FoundationPosition;
        Foundation.setPosition(FoundationPosition);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", FrontleftPower, FrontrightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
 }
 