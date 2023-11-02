package org.firstinspires.ftc.teamcode.commands.autogroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.arm.UnlockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeAdvanceCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.StagedVerticalCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class AutoIntake extends SequentialCommandGroup {

    public AutoIntake(IntakeSubsystem intake){

        addCommands(

                new IntakeOnCommand(intake),
                new WaitCommand(1000),
                new IntakeAdvanceCommand(intake),
                new WaitCommand(1000),
                new IntakeReverseCommand(intake),
                new WaitCommand(1500),
                new IntakeOffCommand(intake)
        );

        addRequirements(intake);
    }
}
