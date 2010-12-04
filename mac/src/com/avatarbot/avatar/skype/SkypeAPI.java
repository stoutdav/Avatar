package com.avatarbot.avatar.skype;

import org.rococoa.NSClass;
import org.rococoa.NSObject;
import org.rococoa.Rococoa;
import org.rococoa.RunOnMainThread;

public interface SkypeAPI extends NSObject {
	
	public static final Skype LIBRARY = Skype.instance;
	public static final _Class CLASS = Rococoa.createClass("SkypeAPI", _Class.class);
	
	public interface _Class extends NSClass {
        boolean isSkypeRunning();
        boolean isSkypeAvailable();
        @RunOnMainThread void setSkypeDelegate(SkypeAPIDelegate aDelegate);
        void connect();
        void disconnect();
        String sendSkypeCommand(String aCommandString);
    }
}
