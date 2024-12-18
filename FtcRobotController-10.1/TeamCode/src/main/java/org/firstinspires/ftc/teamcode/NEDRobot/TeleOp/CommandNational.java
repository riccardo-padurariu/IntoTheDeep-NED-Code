package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@TeleOp(name = "OpModeNational", group = "National")
@Disabled

public class CommandNational extends CommandOpMode {


    VoltageSensor voltageSensor;

    RevColorSensorV3 col1;
    RevColorSensorV3 col2;
    DistanceSensor awd;
    public boolean time_to_intake=true;

    public double curr_pos;
    public static double Q=0.3,R=5;
    public static int N=3;

    public static double G1=0.5,G2=0.5;
    KalmanFilter filter = new KalmanFilter(Q,R,N);
    LowPassFilter ptSensor = new LowPassFilter(0.8);
    LowPassFilter lowPassFilter1 = new LowPassFilter(G1);
    LowPassFilter lowPassFilter2 = new LowPassFilter(G2);
    public double passfil1,passfil2;
    public double filtered_pos;
    public static double targetPosition=4.3;
    public int i=0;

    public enum MODE{
        TELE_OP,
        AUTO
    }
    MODE case1 = MODE.TELE_OP;

    private double loopTime = 0;
  //  private BaseRobot robot;
    boolean timp;

    private  int HighJunctionPos =  1700;
    private  int MidJunctionPos = 1135;
    private  int LowJunctionPos = 580;


    private boolean NUSTRICAINTAKE = false;

    private InstantCommand closeClawCommand;
    private InstantCommand openClawCommand;
    private InstantCommand FourBarIntakeCommand;
    private InstantCommand FourBarDepositCommand;
    private InstantCommand FourBarTransitionIntakeCommand;
    private InstantCommand FourBarJunctionCommand;
    private InstantCommand FourBarTransitionDepositCommand;

    public boolean Scoring;

    private ElapsedTime timer;
    private GamepadEx GamepadEx1;

    boolean stackButton = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

      //  robot = new BaseRobot(hardwareMap,false);
       // robot.reset();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        awd = hardwareMap.get(DistanceSensor.class, "adw");
        col1 =hardwareMap.get(RevColorSensorV3.class,"col1");
        col2 =hardwareMap.get(RevColorSensorV3.class,"col2");

       // robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.INTAKE);

        col1.initialize();
        col2.initialize();
       // FourBarIntakeCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.INTAKE));

     //   FourBarDepositCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT));

     //   FourBarTransitionIntakeCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE));

     //   FourBarJunctionCommand = new InstantCommand(()-> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.JUNCTION));

       // FourBarTransitionDepositCommand  = new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT));

       // openClawCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN));

       // closeClawCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE));

        GamepadEx1 = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    @Override
    public void run() {
      //  robot.read();
        if(gamepad1.touchpad)
            case1=MODE.AUTO;
        else
            case1=MODE.TELE_OP;
        switch(case1)
        {
        //////////////////////////////GAMEPAD1//////////////////////////////////////////////////////////
            case TELE_OP:
        if(timer==null || timer.seconds()==0)
            timer = new ElapsedTime();
        if(timer.seconds()<4)
            timp=false;
        else
            timp=true;
        if (Scoring) {
          //  robot.drive.setWeightedDrivePower(
            ///        new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * 0.3,
            //                dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * 0.3,
            //                -GamepadEx1.getRightX() * 0.3
            //        )
           // );
        } else {
          //  robot.drive.setWeightedDrivePower(
            //        new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.3 : 1),
            //                dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.3 : 1),
            //                -GamepadEx1.getRightX() * (gamepad1.right_trigger > 0.5 ? 0.3 : 1)
         //   //        )
           // );
        }

       // robot.drive.setWeightedDrivePower(
            //    new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.3 : 1),
                       // dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * (gamepad1.right_trigger > 0.5 ? 0.3 : 1),
                     ///   (robot.dr4bSubsystem.isExtended() ? -scale(GamepadEx1.getRightX(), 0.6) * 0.3 : -scale(GamepadEx1.getRightX(), 0.6) * (gamepad1.left_trigger > 0.5 ? 0.3 : 1))
             //   )
       // );


        if (gamepad1.left_bumper) {
            //CommandScheduler.getInstance().schedule(new TeleopIntakePos(robot));
        }
        if (gamepad1.right_bumper) {
         //   schedule(new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.AlignerState.NOT_ALIGNED)));
           // CommandScheduler.getInstance().schedule(new TeleopDepositCommand(robot));
            time_to_intake = true;
            i=0;
            NUSTRICAINTAKE = false;
            Scoring = false;
            time_to_intake = true;
            timer.reset();
        }

        if (gamepad1.left_trigger > 0.5) {
            schedule(FourBarJunctionCommand);
        }


        if (gamepad1.y) {
         //   schedule(new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.AlignerState.ALIGNED)));
          //  schedule(new TeleopExtendDR4BCommand(robot, HighJunctionPos));
            Scoring = true;
            NUSTRICAINTAKE = true;
        }
        if (gamepad1.x) {
         //   schedule(new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.AlignerState.ALIGNED)));
//            Scoring = true;
            NUSTRICAINTAKE = true;
        }

        if (gamepad1.b) {
          //  schedule(new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.AlignerState.ALIGNED)));
          //  schedule(new TeleopExtendDR4BCommand(robot, LowJunctionPos));
            Scoring = true;
            NUSTRICAINTAKE = true;
        }
        if (gamepad1.a) {
         //   schedule(new TeleopRetractCommand(robot));
            Scoring = false;
            NUSTRICAINTAKE = false;
        }


        curr_pos = awd.getDistance(DistanceUnit.CM);
        filtered_pos = ptSensor.estimate(curr_pos);
      //  if (filtered_pos < targetPosition && time_to_intake && robot.dr4bSubsystem.getDr4bPosition()<100 && timp) {
       //     i++;
     //   }

        if (i == 4 && time_to_intake) {
          //  CommandScheduler.getInstance().schedule(new TeleopIntakePos(robot));
            i = 0;
            time_to_intake = false;
        }


        boolean d1DU = gamepad1.dpad_up;
        boolean d1DD = gamepad1.dpad_down;
        boolean d1DL = gamepad1.dpad_left;
        boolean d1DR = gamepad1.dpad_right;


        stackButton = gamepad1.options;

        ///////////// stack////////////////////////////////////

        if (d1DU && !d1DR && !d1DL && stackButton) {
         //   robot.intakeSubsystem.update(IntakeSubsystem.StackState.FIRSTPICK);

        }

        if (d1DR && !d1DU && !d1DL && !NUSTRICAINTAKE && stackButton) {
          //  robot.intakeSubsystem.update(IntakeSubsystem.StackState.SECONDPICK);

        }

        if (d1DL && !d1DR && !d1DD && stackButton) {
         //   robot.intakeSubsystem.update(IntakeSubsystem.StackState.THIRDPICK);

        }


        if (d1DD && !d1DR && !d1DL && stackButton) {
          //  robot.intakeSubsystem.update(IntakeSubsystem.StackState.FOURTHPICK);

        }


        if (gamepad2.left_bumper) {
            schedule(closeClawCommand);
        }


        if (gamepad2.right_bumper) {
            schedule(openClawCommand);
        }

        if (gamepad2.right_trigger > 0.5) {
            schedule(FourBarJunctionCommand);
        }

        if (gamepad2.left_trigger > 0.5) {
            schedule(FourBarDepositCommand);
        }

        if (gamepad2.y) {
           // schedule(new TeleopExtendDR4BCommand(robot, HighJunctionPos));
            Scoring = true;
            NUSTRICAINTAKE = true;
        }
        if (gamepad2.x) {
            //schedule(new TeleopExtendDR4BCommand(robot, MidJunctionPos));
            Scoring = true;
            NUSTRICAINTAKE = true;
        }

        if (gamepad2.b) {
           // schedule(new TeleopExtendDR4BCommand(robot, LowJunctionPos));
            Scoring = true;
            NUSTRICAINTAKE = true;
        }
        if (gamepad2.a) {
         //   schedule(new TeleopRetractCommand(robot));
            Scoring = false;
            NUSTRICAINTAKE = false;
        }


        if (Math.abs(gamepad2.right_stick_x) > 0.3) {
           // robot.intakeSubsystem.setIntakeFactor(Math.pow(gamepad2.right_stick_x, 3));
        }
        break;


            case AUTO:
            if(timer!=null && timer.seconds()!=0)
            timer.reset();
                if(time_to_intake) {
               // if (passfil1 < 0.18 && passfil2 < 0.169)
                 //   robot.drive.setWeightedDrivePower(new Pose2d(-0.23, 0, 0));
              //  else if (passfil1 < 0.18 && passfil2 > 0.17)
                //    robot.drive.setWeightedDrivePower(new Pose2d(-0.23, 0, -0.2));
               // else if (passfil1 > 0.183 && passfil2 < 0.169)
                 //   robot.drive.setWeightedDrivePower(new Pose2d(-0.23, 0, 0.2));
            }
                curr_pos = awd.getDistance(DistanceUnit.CM);
                filtered_pos = ptSensor.estimate(curr_pos);
            //    if (filtered_pos < 5 && time_to_intake && robot.dr4bSubsystem.getDr4bPosition()<100) {
             //       i++;
            //    }
                if (i == 4 && time_to_intake) {
              //      CommandScheduler.getInstance().schedule(new TeleopIntakePos(robot));
                    i = 0;
                    time_to_intake = false;
                }
                break;
    }

        passfil1 = lowPassFilter1.estimate(col1.getLightDetected());
        passfil2 = lowPassFilter2.estimate(col2.getLightDetected());
      //  robot.dr4bSubsystem.loop();

        CommandScheduler.getInstance().run();

   //     robot.write();



       // telemetry.addData("dr4b_ticks",robot.dr4bSubsystem.getDr4bPosition());
      //  telemetry.addData("dr4b_target_pos",robot.dr4bSubsystem.getLiftTargetPosition());
      //  telemetry.addData("dr4b_power",robot.dr4bSubsystem.dr4b_motor.get());
        telemetry.addData("timp",timer.seconds());
        telemetry.addData("filtered pos",filtered_pos);
        telemetry.addData("i=",i);
        telemetry.addData("time to intake",time_to_intake);
       telemetry.addData("PASSFIL1", passfil1);
       telemetry.addData("PASSFIL2", passfil2);


        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();


    }
    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
     //   robot.dr4bSubsystem.dr4b_motor.resetEncoder();
    /*    robot.intakeSubsystem.intake1.setPosition(0);
        robot.intakeSubsystem.intake2.setPosition(0);
        robot.intakeSubsystem.claw.setPosition(0);*/
    }



    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }

}
