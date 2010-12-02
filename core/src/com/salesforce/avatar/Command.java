package com.salesforce.avatar;

public enum Command {
	TurnLeft("Turns to the left"),
	TurnRight("Turns to the right"),
	LookUp("Looks up"),
	LookDown("Looks down"),
	MoveForward("Moves forward"),
	MoveBackward("Moves backward"),
	Status("Requests the status");
	
	private String description;
	
	private Command(String description) {
		this.description = description;
	}
	
	public String getDescription() {
		return description;
	}
}
