package org.firstinspires.ftc.teamcode.subsystems;

public final class Template {
    private static Template instance;

    private Template() {
    }

    public static synchronized Template get() {
        if (instance == null) {
            instance = new Template();
        }
        return instance;
    }
}
