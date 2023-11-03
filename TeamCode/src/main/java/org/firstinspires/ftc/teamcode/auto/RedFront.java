package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.DropPixelCommand;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmDownAuto;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmUpAuto;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.vision.StopStreamingCommand;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.CenterStageVisionProcessor;

@Autonomous(group = "drive")
public class RedFront extends AutoOpBase {

    private BotBuildersMecanumDrive robot;
    private DriveSubsystem drive;

    private VisionSubsystem visionSubsystem;

    private RobotStateSubsystem state;

    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;

    private VerticalSlideSubsystem verticalSubsystem;


    private TrajectorySequenceFollowerCommand forwardFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand rightFollower;

    private TrajectorySequenceFollowerCommand backFollowerLeft;
    private TrajectorySequenceFollowerCommand backFollowerRight;
    private TrajectorySequenceFollowerCommand backFollowerCenter;

    @Override
    public void initialize() {
        robot = new BotBuildersMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(
                robot, null, telemetry);

        state = new RobotStateSubsystem();

        intakeSubsystem = new IntakeSubsystem(hardwareMap, state);
        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        armSubsystem = new ArmSubsystem(hardwareMap, state);
        verticalSubsystem = new VerticalSlideSubsystem(hardwareMap, state);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(15, 62, Math.toRadians(90));

        drive.setPoseEstimate(startingPosition);

        //standard random forward movement.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)
                //push forward, slightly to the right of center
                .lineToSplineHeading(new Pose2d(15,-32, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(15,-42, Math.toRadians(90)))
                //move back ready to make first move
                .lineToSplineHeading(new Pose2d(50, -35, Math.toRadians(180)))
                .build();


        //this is our standard left hand random move
        TrajectorySequence moveToLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(10,32, Math.toRadians(310)))
                .lineToSplineHeading(new Pose2d(10,52, Math.toRadians(310)))
                .lineToSplineHeading(new Pose2d(55,35, Math.toRadians(180)))
                .build();


        //this is our starting right hand random move
        TrajectorySequence moveToRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(0,32, Math.toRadians(220)))
                .lineToSplineHeading(new Pose2d(15,42, Math.toRadians(220)))
                .lineToSplineHeading(new Pose2d(50,30, Math.toRadians(180)))
                .build();

        //these are the three parking positions at the rear of the field
        //duplicated for each location, the starting paths are very similar.
        TrajectorySequence moveToBackDropParkRight = drive.trajectorySequenceBuilder(moveToRight.end())
                .lineToSplineHeading(new Pose2d(55, 60, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropParkLeft = drive.trajectorySequenceBuilder(moveToLeft.end())
                .lineToSplineHeading(new Pose2d(55, 60, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropParkCenter = drive.trajectorySequenceBuilder(moveForward.end())
                .lineToSplineHeading(new Pose2d(55, 60, Math.toRadians(180)))
                .build();


        forwardFollower = new TrajectorySequenceFollowerCommand(drive, moveForward);
        leftFollower = new TrajectorySequenceFollowerCommand(drive, moveToLeft);
        rightFollower = new TrajectorySequenceFollowerCommand(drive, moveToRight);

        backFollowerLeft = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkLeft);
        backFollowerRight = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkRight);
        backFollowerCenter = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkCenter);

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new StopStreamingCommand(visionSubsystem),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        leftFollower,

                                        new ArmUpAuto(armSubsystem, verticalSubsystem, state),
                                        new WaitCommand(500),
                                        new DropPixelCommand(armSubsystem),
                                        new WaitCommand(1500),
                                        new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                        new WaitCommand(1000),
                                        backFollowerLeft
                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                            rightFollower,
                                              //
                                                new ArmUpAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(500),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(1500),
                                                new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(1000),
                                                backFollowerRight
                                        ),
                                        new SequentialCommandGroup(
                                            forwardFollower/*,
                                                new ArmUpAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(500),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(1500),
                                                new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(1000),
                                                backFollowerCenter*/

                                        ),
                                        ()->{
                                            return visionSubsystem.getPosition() == CenterStageVisionProcessor.StartingPosition.RIGHT;
                                        }
                                ),
                                ()-> { return visionSubsystem.getPosition() == CenterStageVisionProcessor.StartingPosition.LEFT;
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