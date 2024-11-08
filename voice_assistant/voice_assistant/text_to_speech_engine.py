import io
import os

import pydub
import pydub.playback
import pyttsx3
from elevenlabslib import *
import base64
# import openai
from pathlib import Path
from openai import OpenAI
client = OpenAI()


class TextToSpeechEngine:
    def __init__(self, name):
        self.name = name

    def speak(self):
        pass


class ElevenLabsEngine(TextToSpeechEngine):
    def __init__(self):
        super().__init__('ElevenLabsTTS')
        user = ElevenLabsUser(os.environ.get('ELEVENLABS_API_KEY'))
        self.voice = user.get_voices_by_name("Sarah")[0]

    def speak(self, msg_list):
        def play(bytesData):
            sound = pydub.AudioSegment.from_file_using_temporary_files(
                io.BytesIO(bytesData))
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
        self.tts_engine.setProperty(
            'voice', voices[11].id)  # 11 is for english

    def speak(self, msg_list):
        if isinstance(msg_list, list):
            for msg in msg_list:
                self.tts_self.speak(msg)
        else:
            self.tts_self.speak(msg)

        self.tts_engine.runAndWait()

# # test = ElevenLabsEngine()


class OpenAITextToSpeechEngine(TextToSpeechEngine):
    def __init__(self):
        super().__init__('OpenAITTS')
        # openai.api_key = os.environ.get('OPENAI_API_KEY')  # Fetch API key from environment variable
        client = OpenAI()
    def speak(self, msg_list):
        import time
        current = time.time()
        # Method to play audio from bytes
        def play(bytes_data):
            sound = pydub.AudioSegment.from_file_using_temporary_files(
                io.BytesIO(bytes_data))
            pydub.playback.play(sound)

        # Function to generate audio using OpenAI TTS API
        def generate_audio(text):
            response = client.audio.speech.create(
                model="tts-1",  # Adjust based on the correct OpenAI model name
                voice="alloy",  # Choose the desired voice (if available in OpenAI)
                input=text
            )
            wav_bytes = response.content
            return wav_bytes  # Assuming the audio bytes are returned in this field

        # Process the message list (either single string or list of strings)
        time_cost = time.time()-current
        print(f"tts engine spends {time_cost} for initializing the play module")
        if isinstance(msg_list, list):
            for msg in msg_list:
                audio_bytes = generate_audio(msg)
                play(audio_bytes)
        elif isinstance(msg_list, str):
            audio_bytes = generate_audio(msg_list)
            play(audio_bytes)
        else:
            play(msg_list)



    # def play_audio(self, bytes_data):
    #     sound = pydub.AudioSegment.from_file_using_temporary_files(
    #         io.BytesIO(bytes_data))
    #     pydub.playback.play(sound)
