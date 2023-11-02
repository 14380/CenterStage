package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ArmLeftCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmLeftCommand(ArmSubsystem arm){
        armSubsystem = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        armSubsystem.ArmLeftPosition();
    }

    @Override
    public boolean isFinished(){

        return this.armSubsystem.IsArmLeft();
    }

}
