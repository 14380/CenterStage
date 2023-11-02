package org.firstinspires.ftc.teamcode.teleop;



        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

        import com.acmerobotics.roadrunner.geometry.Pose2d;
        import com.arcrobotics.ftclib.command.CommandOpMode;
        import com.arcrobotics.ftclib.command.CommandScheduler;

        import com.arcrobotics.ftclib.command.ConditionalCommand;
        import com.arcrobotics.ftclib.command.InstantCommand;
        import com.arcrobotics.ftclib.command.ParallelCommandGroup;
        import com.arcrobotics.ftclib.command.SequentialCommandGroup;
        import com.arcrobotics.ftclib.command.WaitCommand;
        import com.arcrobotics.ftclib.command.button.Trigger;
        import com.arcrobotics.ftclib.gamepad.GamepadEx;
        import com.arcrobotics.ftclib.gamepad.GamepadKeys;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode.commands.arm.ArmExtendoInCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.ArmExtendoOutCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.ArmLeftCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.ArmRightCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.DropPixelCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.LockTransferCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmCenterCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmDownCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmDownSlow1Command;
        import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.PixelCloseCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.RotateCenterCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.UnlockTransferCommand;
        import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.ExtendHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.IncExtendHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.IncOffHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.RetractHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeAdvanceCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeReverseCommand;
        import org.firstinspires.ftc.teamcode.commands.launch.LaunchPlaneCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.ExtendVerticalCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.Pos1ExtendCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.Pos2ExtendCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.RetractVerticalCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.StagedVerticalCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.DeployLeftHookCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.DeployRightHookCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchOffCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchOnCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchReverseCommand;
        import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
        import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

        import java.util.function.BooleanSupplier;

@Config
@TeleOp(group = "drive")
public class BBTeleOp extends CommandOpMode {

    private DriveCommand driveCommand;
    private IntakeSubsystem intakeSubsystem;

    //private HorizontalSlideSubsystem horizontalSlideSubsystem;

    private VerticalSlideSubsystem verticalSlideSubsystem;

    private WinchSubsystem winchSubsystem;

    private ArmSubsystem armSubsystem;
    private LauncherSubsystem launchSubsystem;


    private DriveSubsystem driveSystem;

    private RobotStateSubsystem stateSubsystem;

    private IntakeOnCommand intakeOnCommand;
    private GamepadEx gp1;
    private GamepadEx gp2;
    private BotBuildersMecanumDrive mecDrive;

    public static double transferSetpoint = 0;
    public static double armSetpoint = 0;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mecDrive = new BotBuildersMecanumDrive(hardwareMap);

        stateSubsystem = new RobotStateSubsystem();

        intakeSubsystem = new IntakeSubsystem(hardwareMap, stateSubsystem);
       // horizontalSlideSubsystem = new HorizontalSlideSubsystem(hardwareMap, stateSubsystem);
        winchSubsystem = new WinchSubsystem(hardwareMap, stateSubsystem);
        verticalSlideSubsystem = new VerticalSlideSubsystem(hardwareMap, stateSubsystem);
        armSubsystem = new ArmSubsystem(hardwareMap, stateSubsystem);
        launchSubsystem = new LauncherSubsystem(hardwareMap, stateSubsystem);



        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);




        driveSystem = new DriveSubsystem(
                mecDrive, gp1, telemetry);


        driveCommand = new DriveCommand(
                driveSystem, () -> -gp1.getLeftY(),
                gp1::getLeftX, gp1::getRightX
        );


        schedule(driveCommand);



    }

    @Override
    public void run(){

        //schedule the run after we update our loop.
        CommandScheduler.getInstance().run();

       gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
               //intake on
               new SequentialCommandGroup(
                        new IntakeOnCommand(intakeSubsystem),
                        new LockTransferCommand(armSubsystem)
                       )
       ).whenReleased(
               //intake off.
               new IntakeOffCommand(intakeSubsystem)
       );

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                //intake reversed

                new IntakeReverseCommand(intakeSubsystem)

        ).whenReleased(
                //intake off.
                new IntakeOffCommand(intakeSubsystem)
        );

        //conditional transfer, or extendo when the arm is up
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(

                 new ConditionalCommand(
                        new IntakeAdvanceCommand(intakeSubsystem),
                         new ArmExtendoOutCommand(armSubsystem),
                    () ->{
                        return stateSubsystem.middleArm == RobotStateSubsystem.MiddleArmState.DOWN;
                    }
                )
        );

       /* gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                    new IncExtendHorizontalCommand(horizontalSlideSubsystem),
                    new InstantCommand( () -> {
                        stateSubsystem.horizontalHeight = RobotStateSubsystem.HorizontalHeight.EXTENDED;
                    })
                )
        ).whenReleased(
                new SequentialCommandGroup(
                    new IncOffHorizontalCommand(horizontalSlideSubsystem),
                        new InstantCommand( () -> {
                            stateSubsystem.horizontalHeight = RobotStateSubsystem.HorizontalHeight.EXTENDED;
                        })
                )
        );

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(

            new SequentialCommandGroup(
                    new RetractHorizontalCommand(horizontalSlideSubsystem),
                    new InstantCommand( () -> {
                        stateSubsystem.horizontalHeight = RobotStateSubsystem.HorizontalHeight.RETRACTED;
                    })
            )
        );*/

        gp2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
            new DeployLeftHookCommand(winchSubsystem)
        );

        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new DeployRightHookCommand(winchSubsystem)
        );


        //Collapse the whole arm system depending on state.
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(


                               // new RotateCenterCommand(armSubsystem),
                               // new WaitCommand(80),

                                new ParallelCommandGroup(
                                        new ArmExtendoInCommand(armSubsystem),
                                        new WaitCommand(100),
                                        new RetractVerticalCommand(verticalSlideSubsystem, stateSubsystem),
                                        new SequentialCommandGroup(
                                                new MiddleArmDownSlow1Command(armSubsystem),
                                                new WaitCommand(50),
                                                new MiddleArmDownCommand(armSubsystem),
                                                new PixelCloseCommand(armSubsystem)
                                        )

                                       ),

                        new InstantCommand(() ->{
                            stateSubsystem.middleArm = RobotStateSubsystem.MiddleArmState.DOWN;
                            stateSubsystem.verticalHeight = RobotStateSubsystem.VerticalHeight.DOWN;

                        })
                )

        );

        //only work if the arm is up
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ConditionalCommand(
                    new ArmRightCommand(armSubsystem),
                        new InstantCommand(), //DO nothing
                        () ->{
                        return stateSubsystem.middleArm == RobotStateSubsystem.MiddleArmState.UP;
                        })
        );

        // only work if the arm is up
        /*gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(

            new ConditionalCommand(
                    new ArmLeftCommand(armSubsystem),
                    new InstantCommand(), //DO nothing
                    () ->{
                        return true;//stateSubsystem.middleArm == RobotStateSubsystem.MiddleArmState.UP;
                    })
        );*/


        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
        }).whenActive(new WinchOnCommand(winchSubsystem));


        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
            }
        }).whenActive(new WinchReverseCommand(winchSubsystem));


        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5 && gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5;
            }
        }).whenActive(new WinchOffCommand(winchSubsystem));

        //we first put the arm up,
        //if the arm is already up, then we move the slides up.
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(

                new ConditionalCommand(

                        new SequentialCommandGroup(
                                new UnlockTransferCommand(armSubsystem),
                                new MiddleArmUpCommand(armSubsystem),
                                new InstantCommand(()->{
                                    stateSubsystem.middleArm = RobotStateSubsystem.MiddleArmState.UP;
                                })
                        ),
                        new StagedVerticalCommand(verticalSlideSubsystem, stateSubsystem),
                        () ->{
                            return stateSubsystem.middleArm == RobotStateSubsystem.MiddleArmState.DOWN;
                        })
        );

        //only work if the robot is in the delivery state
        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(

                new DropPixelCommand(armSubsystem)
        );


        //schedule(new InstantCommand(() -> telemetry.addData( "Hor", horizontalSlideSubsystem.getCurrentPosition() )));
        schedule(new InstantCommand(() -> telemetry.addData( "Vert", verticalSlideSubsystem.getCurrentPosition() )));
        schedule(new InstantCommand(() -> telemetry.addData( "Arm State", stateSubsystem.middleArm )));
        schedule(new InstantCommand(() -> telemetry.addData( "Vert State", stateSubsystem.verticalHeight )));


        schedule(new InstantCommand(()-> telemetry.update()));

        //re-align the IMU - will this work with Odo Wheels?
        if(gp1.isDown(GamepadKeys.Button.A) && gp1.isDown(GamepadKeys.Button.B)) {
            mecDrive.setPoseEstimate(new Pose2d(0,0,0));
            telemetry.addData("Align", "DONE");
            telemetry.update();
        }

        if(gp2.isDown(GamepadKeys.Button.X) && gp2.isDown(GamepadKeys.Button.Y)) {
            schedule(new LaunchPlaneCommand(launchSubsystem));
            telemetry.addData("Paper", "Plane");
            telemetry.update();
        }

    }





}

