package com.avatarbot.avatar.skype;

public interface SkypePlatformIntegration {
	
	/**
	 * Connects to Skype.
	 */
	public void connect();
	
	/**
	 * Disconnects from Skype.
	 */
	public void disconnect();
	
	/**
	 * Returns whether Skype is available. 
	 * @return
	 */
	public boolean isConnected();
	
	/**
	 * Specifies the delegate to use.
	 * @param delegate The delegate.
	 */
	public void setDelegate(SkypePlatformIntegrationDelegate delegate);
	
	/**
	 * Send a command.
	 * @param command The command.
	 */
	public void sendCommand(String command);
}
