package org.firstinspires.ftc.teamcode.path;

import android.location.Location;

public interface NavPath {
    int getChar();

    void setChar(int character);

    int getindex();

    Location getNext();

    boolean isDone();
}
