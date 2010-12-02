package com.salesforce.avatar.skype;

import com.sun.jna.Library;
import com.sun.jna.Native;

public interface Skype extends Library {
    public static Skype instance = (Skype) Native.loadLibrary("Skype", Skype.class);
}
