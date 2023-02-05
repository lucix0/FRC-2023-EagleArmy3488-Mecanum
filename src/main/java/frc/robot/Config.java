package frc.robot;


import edu.wpi.first.wpilibj.Filesystem;

import com.moandjiezana.toml.Toml;

import java.io.File;

// Holds important informaion about the robot,
// like which subsystems should be enabled,
public class Config {
    private boolean driveEnabled;
    private boolean extenderEnabled;
    private boolean fourBarEnabled;
    private boolean grabberEnabled;
    private boolean driveControllerConnected;
    private boolean operatorControllerConnected;

    public Config() {
        File configFile = new File(Filesystem.getDeployDirectory(), "config/default.toml");
        Toml config = new Toml().read(configFile);

        driveEnabled = config.getBoolean("driveEnabled");
        extenderEnabled = config.getBoolean("extenderEnabled");
        fourBarEnabled = config.getBoolean("fourBarEnabled");
        grabberEnabled = config.getBoolean("grabberEnabled");
        driveControllerConnected = config.getBoolean("driveControllerConnected");
        operatorControllerConnected = config.getBoolean("operatorControllerConnected");
    }

    public Config(String configName) {
        File configFile = new File(Filesystem.getDeployDirectory(), "config/" + configName + ".toml");
        Toml config = new Toml().read(configFile);

        driveEnabled = config.getBoolean("driveEnabled");
        extenderEnabled = config.getBoolean("extenderEnabled");
        fourBarEnabled = config.getBoolean("fourBarEnabled");
        grabberEnabled = config.getBoolean("grabberEnabled");
        driveControllerConnected = config.getBoolean("driveControllerConnected");
        operatorControllerConnected = config.getBoolean("operatorControllerConnected");
    }

    public boolean getDriveEnabled() {
        return driveEnabled;
    }

    public boolean getExtenderEnabled() {
        return extenderEnabled;
    }

    public boolean getFourBarEnabled() {
        return fourBarEnabled;
    }

    public boolean getGrabberEnabled() {
        return grabberEnabled;
    }

    public boolean getDriveControllerStatus() {
        return driveControllerConnected;
    }

    public boolean getOperatorControllerStatus() {
        return operatorControllerConnected;
    }
}
