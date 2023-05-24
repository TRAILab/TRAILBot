import os
import openai
import speech_recognition as sr


class SpeechRecognizer:
    def __init__(self, mic_index, energy_threshold, timeout, phrase_time_limit, use_whisper=True, dynamic_energy_threshold=False):
        self.speech_recognizer = sr.Recognizer()
        # 11 or 12: self microphone, 13: snowball
        self.mic = sr.Microphone(device_index=mic_index)
        self.speech_recognizer.dynamic_energy_threshold = dynamic_energy_threshold
        self.speech_recognizer.energy_threshold = energy_threshold
        self.use_whisper = use_whisper
        self.timeout = timeout
        self.phrase_time_limit = phrase_time_limit

        openai.api_key = os.environ.get('OPENAI_API_KEY')

    def whisper(self, audio):
        """Transcribe audio using openAI's whisper model 

        Args:
            audio : Speech recognizer object that contains user's input 

        Returns:
            user_input (str): transcribed voice command
        """
        with open('speech.wav', 'wb') as f:
            f.write(audio.get_wav_data())
        speech = open('speech.wav', 'rb')
        wcompletion = openai.Audio.transcribe(
            model="whisper-1",
            file=speech
        )
        user_input = wcompletion['text']
        return user_input

    def get_input(self):
        """ 
        Listen to user's voice and transcribe.
        Uses either openAI's whisper or google to transcribe voice commands to text.

        Returns:
            user_input (str): transcribed voice command
        """
        with self.mic as source:
            print("\nAmbient noise adjust...")
            self.speech_recognizer.adjust_for_ambient_noise(source)
            print("\nListening...")
            try:
                audio = self.speech_recognizer.listen(
                    source, timeout=self.timeout, phrase_time_limit=self.phrase_time_limit)
            except Exception as ex:
                print('Listening exception: ', ex)
                return None

            print("\nTranscribing...")
            try:
                if self.use_whisper:
                    user_input = self.whisper(audio)
                    print('Transcribed from whisper: ', user_input)
                else:
                    user_input = self.speech_recognizer.recognize_google(audio)
                    print('Transcribed from google: ', user_input)
                if user_input == '':
                    return None
                return user_input
            except Exception as ex:
                print('Transcribing Failed!')
                print(ex)
                return None

    @staticmethod
    def list_mics():
        return sr.Microphone.list_microphone_names()
