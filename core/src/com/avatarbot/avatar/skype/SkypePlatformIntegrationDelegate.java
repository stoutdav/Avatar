package com.avatarbot.avatar.skype;

interface SkypePlatformIntegrationDelegate {
	/**
	 * Provides the application name. This is the name that shows-up in the Skype Extras Manager.
	 * @return The application name.
	 */
	public String getClientApplicationName();
	
	/**
	 * Notifies the application of an incoming notification.
	 * @param notification The notification.
	 */
	public void handleNotification(String notification);
}
