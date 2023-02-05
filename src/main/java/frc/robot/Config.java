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

    public Config() {
        File configFile = new File(Filesystem.getDeployDirectory(), "config/default.toml");
        Toml config = new Toml().read(configFile);

        driveEnabled = config.getBoolean("driveEnabled");
        extenderEnabled = config.getBoolean("extenderEnabled");
        fourBarEnabled = config.getBoolean("fourBarEnabled");
        grabberEnabled = config.getBoolean("grabberEnabled");
    }

    public Config(String configName) {
        File configFile = new File(Filesystem.getDeployDirectory(), "config/" + configName + ".toml");
        Toml config = new Toml().read(configFile);

        driveEnabled = config.getBoolean("driveEnabled");
        extenderEnabled = config.getBoolean("extenderEnabled");
        fourBarEnabled = config.getBoolean("fourBarEnabled");
        grabberEnabled = config.getBoolean("grabberEnabled");
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
}
