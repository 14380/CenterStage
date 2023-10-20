package org.firstinspires.ftc.teamcode.teleop;



        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

        import com.arcrobotics.ftclib.command.CommandOpMode;
        import com.arcrobotics.ftclib.command.CommandScheduler;

        import com.arcrobotics.ftclib.command.InstantCommand;
        import com.arcrobotics.ftclib.command.ParallelCommandGroup;
        import com.arcrobotics.ftclib.gamepad.GamepadEx;
        import com.arcrobotics.ftclib.gamepad.GamepadKeys;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.ExtendHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.RetractHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeAdvanceCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeReverseCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.ExtendVerticalCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.RetractVerticalCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchOffCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchOnCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchReverseCommand;
        import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
        import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

@Config
@TeleOp(group = "drive")
public class BBTeleOp extends CommandOpMode {

    private DriveCommand driveCommand;
    private IntakeSubsystem intakeSubsystem;

    private HorizontalSlideSubsystem horizontalSlideSubsystem;

    private VerticalSlideSubsystem verticalSlideSubsystem;

    private WinchSubsystem winchSubsystem;

    private IntakeOnCommand intakeOnCommand;
    private GamepadEx gp1;
    private GamepadEx gp2;
    private BotBuildersMecanumDrive mecDrive;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mecDrive = new BotBuildersMecanumDrive(hardwareMap);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        horizontalSlideSubsystem = new HorizontalSlideSubsystem(hardwareMap);
        winchSubsystem = new WinchSubsystem(hardwareMap);
        verticalSlideSubsystem = new VerticalSlideSubsystem(hardwareMap);


        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);




        DriveSubsystem driveSystem = new DriveSubsystem(
                mecDrive, gp1, telemetry);


        driveCommand = new DriveCommand(
                driveSystem, () -> -gp1.getLeftY(),
                gp1::getLeftX, gp1::getRightX
        );


        schedule(driveCommand);



    }

    @Override
    public void run(){

        CommandScheduler.getInstance().run();


       gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
               //intake on
               new IntakeOnCommand(intakeSubsystem)
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

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ExtendHorizontalCommand(horizontalSlideSubsystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new IntakeAdvanceCommand(intakeSubsystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new RetractHorizontalCommand(horizontalSlideSubsystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new WinchOnCommand(winchSubsystem)
        ).whenReleased(
                new WinchOffCommand(winchSubsystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new WinchReverseCommand(winchSubsystem)
        ).whenReleased(
                new WinchOffCommand(winchSubsystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ExtendVerticalCommand(verticalSlideSubsystem)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
            new RetractVerticalCommand(verticalSlideSubsystem)
        );


        schedule(new InstantCommand(() -> telemetry.addData( "Pos", horizontalSlideSubsystem.getCurrentPosition() )));
        schedule(new InstantCommand(() -> telemetry.addData( "Vert", verticalSlideSubsystem.getCurrentPosition() )));

        //schedule(new InstantCommand(() -> telemetry.addData( "Top", intakeSubsystem.getTopDistance() )));
        //schedule(new InstantCommand(() -> telemetry.addData( "Bottom", intakeSubsystem.getBottomDistance() )));

        schedule(new InstantCommand(()-> telemetry.update()));

        if(gp1.isDown(GamepadKeys.Button.X) && gp1.isDown(GamepadKeys.Button.Y)) {
            mecDrive.ReAlignIMU();
        }

    }



}

