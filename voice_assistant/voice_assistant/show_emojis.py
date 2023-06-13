from pathlib import Path

import cv2


class Emojis():
    def __init__(self) -> None:
        # TODO: is there a cleaner way to get the emojis path?
        self.emoji_path = Path.home() / 'trail_ws/src/TRAILBot/voice_assistant/emojis'
        cv2.startWindowThread()

    def show_speaking(self):
        path = self.emoji_path / 'smile_emoji.png'
        img = cv2.imread(path.__str__())
        cv2.imshow("GUI", img)

    def show_thinking(self):
        path = self.emoji_path / 'think_emoji.png'
        img = cv2.imread(path.__str__())
        cv2.imshow("GUI", img)

    def show_listening(self):
        path = self.emoji_path / 'listen_emoji.png'
        img = cv2.imread(path.__str__())
        cv2.imshow("GUI", img)
