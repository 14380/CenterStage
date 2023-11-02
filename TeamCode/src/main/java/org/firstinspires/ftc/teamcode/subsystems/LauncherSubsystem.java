package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherSubsystem extends SubsystemBase {

    private ServoEx leftServo;
    private ServoEx rightServo;

    private ServoEx planeLauncherServo;

    public LauncherSubsystem(HardwareMap map, RobotStateSubsystem state){
        leftServo = map.get(ServoEx.class, "leftLauncher");
        rightServo = map.get(ServoEx.class, "rightLauncher");

        planeLauncherServo = map.get(ServoEx.class, "planeLaunch");

        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    public void LaunchPlane(){

        planeLauncherServo.setPosition(1);
    }

    public void LeftDeployArm(){

    }

    public void RightDeployArm(){

    }


}
