package org.firstinspires.ftc.teamcode.path;

import android.location.Location;

public interface NavPathInterface {
    int getSegment();

    void setSegment(int segment);

    int getindex();

    Location getNext();

    boolean isDone();

    String getPathName();
}
