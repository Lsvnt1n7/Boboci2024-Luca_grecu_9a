package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.teamcode.drive.robot.Robot; // importeaza biblioteci

@TeleOp(name="MecanumDriveMode", group="Linear OpMode") //declara clasa TeleOp

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null; // declara robotul care cuprinde piesele
    int direction = 1; 
    double servoPosSlides = 0.5;
    double servoPosGrippy = 0;
    //seteaza variabilelel initiale ale servourilor si a directiei
    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * 3 * abs(x);
    } // lucreaza datele primite din controller

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing..."); //adauga un mesaj
        telemetry.update();

        robot = new Robot(hardwareMap); // robotul intra in hardware map
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        } // nu face nimic pana nu e gata 100%

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // telemetria e in dashboard
        telemetry.addData(">", "Initialized"); // cand e gata 100% afiseaza 
        telemetry.update();



        waitForStart(); // asteapta startul
        if (isStopRequested()) return; // opreste daca se cere 


        while (opModeIsActive()) { // cat timp este activ  modul op



            if (gamepad2.left_bumper) { //daca L1 este apasat pe controller 2
                robot.crane.slidesDirection = 1; // misca bratul pozitiv
                robot.crane.setSlides(5); // puterea bratului*
                if(robot.crane.slideEncoderLastPosition > robot.crane.slideEncoder.getVoltage()){
                    robot.crane.slideExtension -= 3.3;
                }
            } else if (gamepad2.right_bumper) { //daca R1 este apasat pe controller 2
                robot.crane.slidesDirection = -1; // misca bratul negativ
                robot.crane.setSlides(5); // puterea bratului
                if(robot.crane.slideEncoderLastPosition < robot.crane.slideEncoder.getVoltage()){
                    robot.crane.slideExtension += 3.3;
                }
            } else {
               robot.crane.setSlides(0); //daca nu este nmk apasat opreste extenisa
            }
            robot.crane.slideEncoderLastPosition = robot.crane.slideEncoder.getVoltage();


            if(gamepad2.left_trigger > 0.1){ // daca L2 este apasat
                robot.crane.craneTarget -= (int) calculateThrottle(gamepad2.left_trigger);
            }
            else if(gamepad2.right_trigger > 0.1){ // altfel daca R2 este apasat
                robot.crane.craneTarget += (int) calculateThrottle(gamepad2.right_trigger);
            }
            robot.crane.motorCrane1.setPower(robot.crane.cranePower(robot.crane.craneTarget));
            robot.crane.motorCrane2.setPower(robot.crane.cranePower(robot.crane.craneTarget));

            if (gamepad2.a) { //daca este abasat X de pe controller2
                robot.crane.gripperDirection = 1;
                robot.crane.setGripper(1);
            }
            else if (gamepad2.b) { // daca este apasat *cerc* de pe controller2
                robot.crane.gripperDirection = -1;
                robot.crane.setGripper(1);
            }
            else robot.crane.setGripper(0);

            robot.drive.setWeightedDrivePower(new Pose2d((-gamepad1.left_stick_y),(-gamepad1.left_stick_x),(-gamepad1.right_stick_x)));








            telemetry.addData("crane target: ", robot.crane.craneTarget);
                telemetry.addData("right trigger: ", gamepad2.right_trigger);
                telemetry.addData("encoder value: ", robot.crane.slideEncoder.getVoltage());
                telemetry.addData("last position ", robot.crane.slideEncoderLastPosition);
                telemetry.addData("slide extension ", robot.crane.slideExtension);
                telemetry.addData("sensor touch: ", robot.crane.slideSensor.isPressed());
//                telemetry.addData("CRANE TICKS LEFT: ", robot.crane.motorCraneLeft.getCurrentPosition());
//                telemetry.addData("CRANE TICKS RIGHT: ", robot.crane.motorCraneRight.getCurrentPosition());
//                telemetry.addData("DIRECTION: ", direction);
//                telemetry.addData("SERVO GRIPPER: ", robot.crane.servoGrippy1.getPosition());
                telemetry.update();
            }

        }

    }



