package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.DropPixelCommand;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmDownAuto;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmUpAuto;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.horizontal.ZeroOffHorizontalCommand;
import org.firstinspires.ftc.teamcode.commands.intake.RetractPurpleCommand;
import org.firstinspires.ftc.teamcode.commands.vision.StopStreamingCommand;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.CenterStageVisionProcessor;

@Autonomous(group = "drive")
public class BlueBackdropYellowPark extends AutoOpBase {

    private BotBuildersMecanumDrive robot;
    private DriveSubsystem drive;

    private VisionSubsystem visionSubsystem;

    private RobotStateSubsystem state;

    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;

    private HorizontalSlideSubsystem horizontalSlideSubsystem;

    private VerticalSlideSubsystem verticalSubsystem;


    private TrajectorySequenceFollowerCommand forwardFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand rightFollower;

    private TrajectorySequenceFollowerCommand movePixelToCenterFollower;
    private TrajectorySequenceFollowerCommand movePixelToLeftFollower;
    private TrajectorySequenceFollowerCommand movePixelToRightFollower;




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
        horizontalSlideSubsystem = new HorizontalSlideSubsystem(hardwareMap, state, intakeSubsystem);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(15, 62, Math.toRadians(270));

        drive.setPoseEstimate(startingPosition);

        //standard random forward movement. for center
        TrajectorySequence moveForwardCenter = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(61,23.5, Math.toRadians(180)))
                .build();

        //simple movement to move close to the backdrop
        TrajectorySequence movePixelToCenter = drive.trajectorySequenceBuilder(moveForwardCenter.end())
                .lineToSplineHeading(new Pose2d(30,14, Math.toRadians(180)))
                .build();

        //move into the center of the field, ready to go down to the stack,
        //this is a common position for all of the routes (left, center, right).
        TrajectorySequence moveIntoCenterPosition = drive.trajectorySequenceBuilder(movePixelToCenter.end())
                .lineToSplineHeading(new Pose2d(55, 58, Math.toRadians(265))) //PARK CENTER

                .build();

        //this is our standard left hand random move
        TrajectorySequence moveToLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(61,29.5, Math.toRadians(180)))
                .build();

        TrajectorySequence movePixelToLeft = drive.trajectorySequenceBuilder(moveToLeft.end())
                .lineToSplineHeading(new Pose2d(38,16, Math.toRadians(180)))
                .build();

        TrajectorySequence moveLeftIntoMovePos = drive.trajectorySequenceBuilder(movePixelToLeft.end())
                //.lineToSplineHeading(new Pose2d(50,16, Math.toRadians(180)))
               // .lineToSplineHeading(new Pose2d(55, -1, Math.toRadians(175))) // PARK LEFT
                .lineToSplineHeading(new Pose2d(55, 58, Math.toRadians(265)))
                .build();


        //this is our starting right hand random move
        TrajectorySequence moveToRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(61,17, Math.toRadians(180)))
                .build();

        TrajectorySequence movePixelToRight = drive.trajectorySequenceBuilder(moveToLeft.end())
                .lineToSplineHeading(new Pose2d(20,16, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(18,16, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence moveToCenterRight = drive.trajectorySequenceBuilder(movePixelToRight.end())
              //  .lineToSplineHeading(new Pose2d(55, -1, Math.toRadians(175))) //PARK RIGHT
                .lineToSplineHeading(new Pose2d(55, 58, Math.toRadians(265)))
                .build();




        forwardFollower = new TrajectorySequenceFollowerCommand(drive, moveForwardCenter);
        leftFollower = new TrajectorySequenceFollowerCommand(drive, moveToLeft);
        rightFollower = new TrajectorySequenceFollowerCommand(drive, moveToRight);


        movePixelToCenterFollower = new TrajectorySequenceFollowerCommand(drive, movePixelToCenter);
        movePixelToLeftFollower = new TrajectorySequenceFollowerCommand(drive, movePixelToLeft);
        movePixelToRightFollower = new TrajectorySequenceFollowerCommand(drive, movePixelToRight);

        //these are the three centering positions.
        TrajectorySequenceFollowerCommand moveLeftToMovePosition = new TrajectorySequenceFollowerCommand(drive, moveLeftIntoMovePos);
        TrajectorySequenceFollowerCommand moveToCenterFollower = new TrajectorySequenceFollowerCommand(drive, moveIntoCenterPosition);
        TrajectorySequenceFollowerCommand moveToCenterRightFollower = new TrajectorySequenceFollowerCommand(drive, moveToCenterRight);

        //load the pixel pusher


        intakeSubsystem.ExtendPurple();

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new StopStreamingCommand(visionSubsystem),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                leftFollower,
                                                new SequentialCommandGroup(
                                                        new WaitCommand(500),
                                                        new ArmUpAuto(armSubsystem, verticalSubsystem, state)
                                                )
                                        ),
                                         new WaitCommand(500),
                                        new WaitCommand(300),
                                        new DropPixelCommand(armSubsystem),
                                        new WaitCommand(250),
                                        new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                        new WaitCommand(500),
                                        movePixelToLeftFollower,
                                        new RetractPurpleCommand(intakeSubsystem),
                                        moveLeftToMovePosition

                                        ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        rightFollower,
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500),
                                                                new ArmUpAuto(armSubsystem, verticalSubsystem, state)
                                                        )
                                                ),
                                                new WaitCommand(500),
                                                new WaitCommand(300),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(250),
                                                new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(500),
                                                movePixelToRightFollower,
                                                new RetractPurpleCommand(intakeSubsystem),
                                                moveToCenterRightFollower

                                        ),
                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                    forwardFollower,
                                                    new SequentialCommandGroup(
                                                            new WaitCommand(500),

                                                        new ArmUpAuto(armSubsystem, verticalSubsystem, state)
                                                    )
                                                ),
                                                new WaitCommand(500),

                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(250),
                                                new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(500),
                                                movePixelToCenterFollower,
                                                new RetractPurpleCommand(intakeSubsystem),
                                                new ZeroOffHorizontalCommand(horizontalSlideSubsystem),
                                                moveToCenterFollower

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
