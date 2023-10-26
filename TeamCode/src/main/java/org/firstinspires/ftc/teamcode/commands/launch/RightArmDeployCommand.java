package org.firstinspires.ftc.teamcode.commands.launch;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class RightArmDeployCommand extends CommandBase {

    private LauncherSubsystem launcherSubsystem;

    public RightArmDeployCommand(LauncherSubsystem launch){
        launcherSubsystem = launch;
    }

    @Override
    public void initialize(){

        this.launcherSubsystem.RightDeployArm();
    }

}
