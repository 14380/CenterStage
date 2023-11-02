package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmExtendoInCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmExtendoInCommand(ArmSubsystem arm) {
        armSubsystem = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {

        armSubsystem.ExtendoIn();
    }

    @Override
    public boolean isFinished(){

        return true;
    }
}