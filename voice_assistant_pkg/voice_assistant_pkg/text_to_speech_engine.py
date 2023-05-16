import io
import os

import pydub
import pydub.playback
import pyttsx3
from elevenlabslib import *


class TextToSpeechEngine:
    def __init__(self, name):
        self.name = name
    
    def speak(self):
        pass

class ElevenLabsEngine(TextToSpeechEngine):
    def __init__(self):
        super().__init__('ElevenLabsTTS')
        user = ElevenLabsUser(os.environ.get('ELEVENLABS_API_KEY'))
        self.voice = user.get_voices_by_name("Rachel")[0] 
    
    def speak(self, msg_list):
        def play(bytesData):
            sound = pydub.AudioSegment.from_file_using_temporary_files(io.BytesIO(bytesData))
            pydub.playback.play(sound)
            return
        
        if isinstance(msg_list, list):
            for msg in msg_list:
                play(self.voice.generate_audio_bytes(msg))
        else:
            play(self.voice.generate_audio_bytes(msg_list))

class Pyttsx3Engine(TextToSpeechEngine):
    def __init__(self):
        super().__init__('Pyttsx3')
        self.tts_engine = pyttsx3.init()
        voices = self.tts_engine.getProperty('voices')
        self.tts_engine.setProperty('voice', voices[11].id) # 11 is for english
    
    def speak(self, msg_list):
        if isinstance(msg_list, list):
            for msg in msg_list:
                self.tts_self.speak(msg)
        else:
            self.tts_self.speak(msg)
        
        self.tts_engine.runAndWait()
        
        
    


