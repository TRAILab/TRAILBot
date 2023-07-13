from pathlib import Path
import numpy as np
import cv2

remote_control_laptop_screen_width = 1920

class Emojis():
    def __init__(self, logger) -> None:
        # TODO: is there a cleaner way to get the emojis path?
        self.emoji_path = Path.home() / 'trail_ws/src/TRAILBot/voice_assistant/emojis'
        self.logger = logger
        logger.info(self.emoji_path.__str__())
        cv2.startWindowThread()
        cv2.namedWindow("GUI", cv2.WINDOW_NORMAL)
        # Move window to the 2nd monitor
        cv2.moveWindow("GUI", remote_control_laptop_screen_width+1,0)
        # Set the window properties to full screen
        cv2.setWindowProperty("GUI", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def show_speaking(self):
        path = self.emoji_path / 'smile_emoji.png'
        img = cv2.imread(path.__str__())
        assert img is not None, 'Image is None!'
        cv2.imshow("GUI", img)
        cv2.waitKey(1)

    def show_thinking(self):
        path = self.emoji_path / 'think_emoji.png'
        img = cv2.imread(path.__str__())
        assert img is not None, 'Image is None!'
        cv2.imshow("GUI", img)
        cv2.waitKey(1)

    def show_listening(self):
        path = self.emoji_path / 'listen_emoji.png'
        img = cv2.imread(path.__str__())
        assert img is not None, 'Image is None!'
        cv2.imshow("GUI", img)
        cv2.waitKey(1)
