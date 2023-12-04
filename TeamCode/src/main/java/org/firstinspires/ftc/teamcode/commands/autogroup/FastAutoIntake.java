package org.firstinspires.ftc.teamcode.commands.autogroup;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.arm.LockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.arm.UnlockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeAdvanceCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class FastAutoIntake extends SequentialCommandGroup {

    public FastAutoIntake(IntakeSubsystem intake, ArmSubsystem arm){

        addCommands(
                new LockTransferCommand(arm),
                new IntakeOnCommand(intake),
                new WaitCommand(800),
               // new IntakeOffCommand(intake),
                //new WaitCommand(100),
                //new IntakeAdvanceCommand(intake),
                //new WaitCommand(500),
                new IntakeReverseCommand(intake),
                new WaitCommand(1000),
                new IntakeOffCommand(intake),
                new UnlockTransferCommand(arm)
        );

        addRequirements(intake, arm);
    }
}
