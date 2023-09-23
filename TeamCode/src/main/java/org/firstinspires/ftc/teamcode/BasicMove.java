package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.HashMap;

@Autonomous(group = "drive")
public class BasicMove extends AutoOpBase {

    //private DriveSubsystem drive;


   // private TrajectorySequenceFollowerCommand parkFollower;

    private MecanumDrive robot;

    @Override
    public void initialize() {
        //robot = new MecanumDrive();


        //drive.setPoseEstimate(new Pose2d(72, 0, Math.toRadians(0)));

        /*TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(72, 0, Math.toRadians(0)))

                .setReversed(true)
                .lineToSplineHeading(new Pose2d(62, 0, Math.toRadians(93)))
                .lineToSplineHeading(new Pose2d(11, 0, Math.toRadians(93)))
                .lineToSplineHeading(new Pose2d(11, -11.5, Math.toRadians(93)))
                .build();*/



        //parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);


        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        //if we need to wait first up
                        // new WaitCommand(3000), //three seconds

                               //s parkFollower

                        )
        );



    }

    @Override
    public void run(){

        CommandScheduler.getInstance().run();

    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart(){
        //vision.disable();
    }

}
