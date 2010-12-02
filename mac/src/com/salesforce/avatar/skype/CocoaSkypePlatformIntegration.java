package com.salesforce.avatar.skype;

import org.rococoa.Rococoa;
import org.rococoa.cocoa.foundation.NSAutoreleasePool;
import org.rococoa.cocoa.foundation.NSNotification;


public class CocoaSkypePlatformIntegration implements SkypePlatformIntegration {
	
	private SkypePlatformIntegrationDelegate delegate;

	@Override
	public void connect() {
		SkypeAPI.CLASS.connect();
	}

	@Override
	public void disconnect() {
		SkypeAPI.CLASS.disconnect();
	}

	@Override
	public boolean isConnected() {
		return SkypeAPI.CLASS.isSkypeRunning() && SkypeAPI.CLASS.isSkypeAvailable();
	}

	@Override
	public void setDelegate(SkypePlatformIntegrationDelegate delegate) {
		this.delegate = delegate;
		SkypeAPIDelegate proxy = Rococoa.proxy(this, SkypeAPIDelegate.class);
		SkypeAPI.CLASS.setSkypeDelegate(proxy);
	}

	@Override
	public void sendCommand(String command) {
		// TODO Auto-generated method stub
	}

	public String clientApplicationName() {
		return delegate.getClientApplicationName();
	}

	public void skypeNotificationReceived(String aNotificationString) {
		delegate.handleNotification(aNotificationString);
	}

	public void skypeAttachResponse(int aAttachResponseCode) {
		System.out.println("skypeAttachResponse = " + aAttachResponseCode);
	}
	
	public void skypeBecameAvailable(NSNotification aNotification) {
		System.out.println("skypeBecameAvailable");
	}

	public void skypeBecameUnavailable(NSNotification aNotification) {
		System.out.println("skypeBecameUnavailable");
	}

	public static void main(String[] args) {
		NSAutoreleasePool pool = NSAutoreleasePool.new_();
		try {
			CocoaSkypePlatformIntegration integration = new CocoaSkypePlatformIntegration();
			integration.setDelegate(new SkypePlatformIntegrationDelegate() {
				
				@Override
				public void handleNotification(String notification) {
					System.out.println("notification: " + notification);
				}
				
				@Override
				public String getClientApplicationName() {
					return "test";
				}
			});
			if (integration.isConnected()) {
				integration.connect();
			}
		}
		finally {
			pool.release();
		}
	}
}
