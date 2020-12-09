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

package org.firstinspires.ftc.teamcode.robots.beachcomber;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.LocationTrack;
import org.firstinspires.ftc.teamcode.path.PathLogger;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

import static org.firstinspires.ftc.teamcode.util.Conversions.nearZero;
import static org.firstinspires.ftc.teamcode.util.Conversions.notdeadzone;


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

/**
 * Controls:
 * Path Selection:
 * dpad_right - select next path
 * dpad_left - select previous path
 * a - load selected path
 * x - create new path
 *
 * Main Loop:
 * left_stick_y - drive
 * right_stick_x - turn
 * a - add current position to path
 * b - calibrate IMU from GPS heading - test run of 5 meters
 * y - run path from current location todo
 */

@TeleOp(name="Beach Comber", group="Linear Opmode")
// @Disabled
@Config
public class beachy extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private PathLogger pathLogger;
    private boolean pathSelected = false;
    private LocationTrack locationTrack;
    private StickyGamepad stickyGamepad1;
    static public int nextPath = 0; //blank means don't set a new path - expect this to be set directly by typing into dashboard


    private PoseSkystone.RobotType currentBot = PoseSkystone.RobotType.Beachcomber;

    private PoseSkystone robot;

    private Autonomous auto;

    private boolean active = true;
    private boolean joystickDriveStarted = false;

    static public int state = 0;

    // drive control variables
    private double pwrDamper = 1;
    private double pwrFwd = 0;
    private double pwrRot = 0;
    private int direction = 1; // -1 to reverse direction
    int reverse = 1; //todo: get rid of this and simplify back down to using just direction

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new PoseSkystone(currentBot);
        robot.init(this.hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        //locationTrack = new LocationTrack(AppUtil.getInstance().getActivity());
        //pathLogger = new PathLogger(locationTrack);
        stickyGamepad1 = new StickyGamepad(gamepad1);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*
        List<String> pathNames = pathLogger.getPaths();
        int selectedIndex = 0;
        while(!pathSelected) {
            telemetry.addData("Status", "Selecting path (choose a path with dpad_right, dpad_left, and a, or create a new path with x).");
            if(pathNames.size() > 0) {
                String pathTelemetry = "";
                for(String pathName: pathNames)
                    pathTelemetry += pathName.equals(pathNames.get(selectedIndex)) ? String.format(" **%s** ", pathName) : pathName;
                telemetry.addData("Paths", pathTelemetry);

                if(selectedIndex > 0 && stickyGamepad1.dpad_left)
                    selectedIndex--;
                else if (selectedIndex < pathNames.size() - 1 && stickyGamepad1.dpad_right)
                    selectedIndex++;
            } else
                telemetry.addData("Paths", "No paths created yet.");

            if(stickyGamepad1.a) {
                try {
                    pathLogger.loadPath(pathNames.get(selectedIndex));
                    pathSelected = true;
                } catch(IOException e) {
                    telemetry.addData("Paths", "Pathname not found: " + e);
                }
            } else if (stickyGamepad1.x) {
                try {
                    pathLogger.createPath("", true);
                    pathSelected = true;
                } catch(IOException e) {
                    telemetry.addData("Paths", "Cannot create new path: " + e);
                }
            }

            stickyGamepad1.update();
            telemetry.update();
        }

         */

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
/*
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            */
            JoystickDrive();

            //if(!pathLogger.getReading() && stickyGamepad1.a) pathLogger.addLocation();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", robot.motorBackLeft.getPower(), robot.motorBackRight.getPower());
            telemetry.addData("Position", "latitude (%.7f), longitude (%.7f)", robot.getLatitude(), robot.getLongitude());
            telemetry.addData("Path: ", "%s, segment (%d), index (%d)", robot.getActivePathName(), robot.getSegment(), robot.getindex());
            telemetry.addData("Target distance: ", "(%.2f)m", robot.getDistanceNext());
            telemetry.addData("heading", "degrees (%.2f)", robot.getHeading());
            telemetry.addData("voltage", "%.1f volts", getBatteryVoltage());
            telemetry.addData("imuTemp", "%.1f volts", robot.getTemperature());
            telemetry.update();
            stickyGamepad1.update();
            robot.updateSensors(true);
        }

        pathLogger.savePath();
    }

private void JoystickDrive(){
    if (!joystickDriveStarted) {
        robot.resetMotors(true);
        robot.setAutonSingleStep(true);
        joystickDriveStarted = true;
    }

    //press b to run the IMU to GPS alignment routine
    if (stickyGamepad1.b || nextPath==8) robot.articulate(PoseSkystone.Articulation.alignIMUtoGPS);

    //press x to run the currently set navigation route - be sure alignIMUtoGPS has completed
    if (stickyGamepad1.x || nextPath==9) robot.articulate(PoseSkystone.Articulation.navigate);

    if(stickyGamepad1.dpad_left || nextPath==1) robot.setActivePath(PoseSkystone.Paths.TeamNumber);
    if(stickyGamepad1.dpad_right || nextPath==2) robot.setActivePath(PoseSkystone.Paths.DPRG);
    if(stickyGamepad1.dpad_up || nextPath==3) robot.setActivePath(PoseSkystone.Paths.Square);
    if(stickyGamepad1.dpad_down || nextPath==4) robot.setActivePath(PoseSkystone.Paths.Stay);
    if(nextPath==5) robot.setActivePath(PoseSkystone.Paths.PingPong);
    if(nextPath==6) robot.setActivePath(PoseSkystone.Paths.Infinity);

    nextPath=0; //reset nextPath

    reverse = -1;
    pwrDamper = .70;

    pwrFwd = 0;
    pwrRot = 0;

    if (notdeadzone(gamepad1.left_stick_y))
        pwrFwd = reverse * direction * pwrDamper * gamepad1.left_stick_y;
    if (notdeadzone(gamepad1.right_stick_x))
        pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;

    if (nearZero(pwrFwd) && nearZero(pwrRot) && robot.isNavigating) {
    } else {
        robot.isNavigating = false; // take control back from any auton navigation if any joystick input is running
        //robot.articulate(PoseSkystone.Articulation.manual); //todo: might lead to bad consequences when state isn't cleaned up in the articulation we just aborted
        robot.autonTurnInitialized = false;
        robot.driveMixerDiffSteer(pwrFwd * pwrDamper, pwrRot);
    }
}
    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
