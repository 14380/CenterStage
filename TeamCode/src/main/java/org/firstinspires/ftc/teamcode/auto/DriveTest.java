package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class DriveTest extends AutoOpBase {

    private BotBuildersMecanumDrive robot;
    private DriveSubsystem drive;


    private TrajectorySequenceFollowerCommand parkFollower;

    @Override
    public void initialize() {
        robot = new BotBuildersMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(
                robot, null, telemetry);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startingPosition);

        //Define the movements of the robot that we need.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)
                .forward(11)
                .build();

        parkFollower = new TrajectorySequenceFollowerCommand(drive, moveForward);

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        parkFollower,
                        new InstantCommand( () -> {
                            telemetry.addData("x", drive.getPoseEstimate().getX());
                            telemetry.addData("y", drive.getPoseEstimate().getY());
                            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
                            telemetry.update();
                        })
                ));

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

    }
}
