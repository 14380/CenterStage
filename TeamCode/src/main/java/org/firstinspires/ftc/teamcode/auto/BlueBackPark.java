package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
@Disabled
public class BlueBackPark extends AutoOpBase {

    private BotBuildersMecanumDrive robot;
    private DriveSubsystem drive;

    private RobotStateSubsystem state;


    private TrajectorySequenceFollowerCommand parkFollower;
    private TrajectorySequenceFollowerCommand backFollower;

    private TrajectorySequenceFollowerCommand frontFollower;

    @Override
    public void initialize() {
        robot = new BotBuildersMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(
                robot, null, telemetry);

        state = new RobotStateSubsystem();

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, state);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(-36, 62, Math.toRadians(270));

        drive.setPoseEstimate(startingPosition);

        //Define the movements of the robot that we need.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(-45,0, Math.toRadians(180)))
                .build();



        TrajectorySequence moveToBack = drive.trajectorySequenceBuilder(moveForward.end())
                .lineToSplineHeading(new Pose2d(54, 4, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToFront = drive.trajectorySequenceBuilder(moveToBack.end())
                .lineToSplineHeading(new Pose2d(-45, 4, Math.toRadians(180)))
                .build();


        parkFollower = new TrajectorySequenceFollowerCommand(drive, moveForward);
        backFollower = new TrajectorySequenceFollowerCommand(drive, moveToBack);
        frontFollower = new TrajectorySequenceFollowerCommand(drive, moveToFront);

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        parkFollower,

                        new IntakeOnCommand(intake),
                        new WaitCommand(2500),
                        new IntakeReverseCommand(intake),
                        new WaitCommand(1000),
                        new IntakeOffCommand(intake),

                        backFollower,
                        frontFollower,

                        new IntakeOnCommand(intake),
                        new WaitCommand(2500),
                        new IntakeReverseCommand(intake),
                        new WaitCommand(1000),
                        new IntakeOffCommand(intake),
                        backFollower,

                        frontFollower,

                        new IntakeOnCommand(intake),
                        new WaitCommand(2500),
                        new IntakeReverseCommand(intake),
                        new WaitCommand(1000),
                        new IntakeOffCommand(intake),
                        backFollower

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
