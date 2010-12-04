package com.avatarbot.avatar;

import org.jivesoftware.smack.Chat;
import org.jivesoftware.smack.ChatManagerListener;

/**
 *
 */
public class AvatarChatManagerListener implements ChatManagerListener {
    private AvatarMessageListener messageListener;

    public AvatarChatManagerListener() {
        messageListener = new AvatarMessageListener();
    }

    public void chatCreated(Chat chat, boolean b) {
        chat.addMessageListener(messageListener);
        messageListener.setChat(chat);
    }

    public AvatarMessageListener getMessageListener() {
        return messageListener;
    }

    public void setSerialInterface(AvatarSerialInterface serialInterface) {
        this.messageListener.setSerialInterface(serialInterface);
    }
}