import json
import os
import re
import time
import io

from elevenlabslib import *
import pydub
import pydub.playback
import openai
#import pyttsx3
import rclpy
import speech_recognition as sr
from rclpy.node import Node
from std_msgs.msg import String, Bool

from trailbot_interfaces.srv import SnackWanted

      
class VoiceAssistant(Node):
    def __init__(self, elevenlabs_key='7163ecf62502641ce6ad48a7e808e8d6', openai_api_key='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', snack_options=['chips', 'chocolate', 'nuts', 'candies', 'juice'], use_whisper=True, list_mics=False):
        """
        Listens and comprehends user's commands,
        Sends snack_wanted request to behaviour planner,
        Chats with user using chatGPT

        Args:
            openai_api_key (str): openAI secret api key
            use_whisper (bool, optional): Set to True if you want to use openai transcriber otherwise it uses google transcriber. Defaults to True.
        """
        super().__init__('voice_assistant_node')
        
        self.use_whisper=use_whisper
        self.openai_api_key=openai_api_key
        self.exit_cmd_options=['bye', 'bubye', 'adios', 'ciao', 'thanks', 'thank you']
        self.snack_options=snack_options
        self.snack_options_str=''
        for i in range(len(snack_options)-1):
            self.snack_options_str+= f' {snack_options[i]},'
        self.snack_options_str = self.snack_options_str[:-1] + f' and {snack_options[-1]}' 
        print(self.snack_options_str)

        # openAI set-up
        personality = "You are a helpful assistant."
        openai.api_key = openai_api_key
        self.messages  = [{"role": "system", "content": f"{personality}"}]

        # Text to Speech Generator set-up
        # TODO: Replace this with more realistic speech generator
        # self.tts_engine = pyttsx3.init()
        # voices = self.tts_engine.getProperty('voices')
        # self.tts_engine.setProperty('voice', voices[11].id) # 11 is for english
        user = ElevenLabsUser(elevenlabs_key) #fill in your api key as a string
        self.voice = user.get_voices_by_name("Rachel")[0]  #fill in the name of the voice you want to use. ex: "Rachel"

        # Speech Recognizer set-up
        self.speech_recognizer = sr.Recognizer()
        # Note: You can list microphones with this command and set the device_index 
        # to the index of your favourite microphone from the list
        if list_mics:
            print(sr.Microphone.list_microphone_names())
        self.mic = sr.Microphone(device_index=12) # 11 or 12: self microphone, 13: snowball
        self.speech_recognizer.dynamic_energy_threshold=False
        self.speech_recognizer.energy_threshold = 200

        # Set-up save folder to save conversation
        script_dir = os.path.dirname(os.path.abspath(__file__)) # grab script location
        self.save_foldername = os.path.join(script_dir,f"conversations")
        self.suffix = self.save_conversation()


        # Service
        # Initialize snack_wanted client and request
        self.cli = self.create_client(SnackWanted, 'snack_wanted')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('snack_wanted service not available, waiting again...')
        self.snack_wanted_request = SnackWanted.Request()
        self.snack_wanted = None
        self.want_snacks = False   # Whether user wants snacks or wants to chat
        self.bye_found = False
        self.active = False


        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'state',
            self.listener_callback,
            1)
        self.subscription  # To prevent unused variable warning

        #Publisher
        self.publisher_ = self.create_publisher(Bool, 'chat_ended', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_timer_callback)

    def publisher_timer_callback(self):
        msg = Bool()
        msg.data = self.bye_found
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)

    def listener_callback(self, msg):
        # activate chatbot if in 'Query' state
        if msg.data == 'Query': 
            self.active = True
            self.get_logger().info('Subscribing: "%s"' % msg.data)
            # # Introduce upon reaching the human
            # intro = ['Hi! Would you like to have snacks?',
            #         'Say "snacks", if you would like to have snacks.',
            #         'Otherwise, you can ask me any question.']
            
            # self.speak(intro)

            # # Get user prompt
            # user_input = self.get_input()

            # # Repeat intro if user did not say anything
            # if user_input is None:
            #     return
            
            # self.want_snacks, _ = self.look_for_keywords(user_input, ['snacks'] + self.snack_options)

            # if self.want_snacks:
            #     if self.get_snack_wanted():
            #         self.request_snacks()
            # else:
            #     self.chat_with_user(user_input)
            
            # time.sleep(5)
            # self.bye_found = False
        else:
            self.active = False


            

                
    def send_request(self):
        """
        Sends request to snack_wanted service 

        Args:
            snack (str): snack requested by user e.g. 'chips'

        Returns:
            response (bool, str): response of snack_wanted service 
                                    e.g. response.success = 'True', 
                                    response.message = 'Snack successfully dispensed'
        """

        self.snack_wanted_request.snack = self.snack_wanted
        self.future = self.cli.call_async(self.snack_wanted_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def request_snacks(self):
        self.speak(f'Alrighty, {self.snack_wanted} coming right up!')

        # Run Servo and Dispense Snack
        response = self.send_request()
        # Print Response
        self.get_logger().info(f'Response of snack_wanted service request: {response.success}, {response.message}')

        if response.success:
            self.speak(f'Please pick up your {self.snack_wanted} from my side!')
            self.speak(f'Thank you! Have a nice Day!')
        else:
            self.speak(f'Oops, there seems to be a problem!') # TODO: what to do in this case?
    
    def whisper(self, audio):
        """Transcribe audio using openAI's whisper model 

        Args:
            audio : Speech recognizer object that contains user's input 

        Returns:
            user_input (str): transcribed voice command
        """
        with open('speech.wav','wb') as f:
            f.write(audio.get_wav_data())
        speech = open('speech.wav', 'rb')
        wcompletion = openai.Audio.transcribe(
            model = "whisper-1",
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
            audio = self.speech_recognizer.listen(source)
            try:
                if self.use_whisper:
                    user_input = self.whisper(audio)
                    print('Transcribed from whisper: ', user_input)
                else:
                    user_input = self.speech_recognizer.recognize_google(audio)
                    print('Transcribed from google: ', user_input)
                return user_input
            except:
                return None
    
    def save_conversation(self):
        '''
        Checks the folder for previous conversations and will get the next suffix that has not been used yet.  
        It returns suffix number.

        Args:
            save_foldername (str) : Takes in the path to save the conversation to.
            messages (list of dicts): conversation 
        
        Returns:
            suffix (int): suffix number to label the current conversation .txt file 
        '''
        
        os.makedirs(self.save_foldername, exist_ok=True)

        base_filename = 'conversation'
        suffix = 0
        filename = os.path.join(self.save_foldername, f'{base_filename}_{suffix}.txt')

        while os.path.exists(filename):
            suffix += 1
            filename = os.path.join(self.save_foldername, f'{base_filename}_{suffix}.txt')

        with open(filename, 'w') as file:
            json.dump(self.messages, file, indent=4)

        return suffix

    def save_inprogress(self):
        '''
        Uses the suffix number returned from save_conversation to continually update the 
        file for this instance of execution.  This is so that you can save the conversation 
        as you go so if it crashes, you don't lose to conversation.

        Args:
            suffix  :  Takes suffix count from save_conversation()
        '''
        os.makedirs(self.save_foldername, exist_ok=True)
        base_filename = 'conversation'
        filename = os.path.join(self.save_foldername, f'{base_filename}_{self.suffix}.txt')

        with open(filename, 'w') as file:
            json.dump(self.messages, file, indent=4)

    def speak(self, msg_list):
        """ Text to speech generation

        Args:
            msg_list (list of str): list of sentences to speak
        """
        # for msg in msg_list:
        #     self.tts_self.speak(msg)
        # self.tts_engine.runAndWait()

        def play(bytesData):
            sound = pydub.AudioSegment.from_file_using_temporary_files(io.BytesIO(bytesData))
            pydub.playback.play(sound)
            return
        
        if isinstance(msg_list, list):
            for msg in msg_list:
                play(self.voice.generate_audio_bytes(msg))
        else:
            play(self.voice.generate_audio_bytes(msg_list))

    def look_for_keywords(self, user_input, keywords):
        """ Look for keywords in user's prompt
        Upon detection return True and the word found

        Args:
            user_input (str): transcribed voice command
            keywords (list of str): list of keywords to look for
        Return:
            bool: if any keyword is found in user_input
            word (str): keyword found
        """

        # Make capital letters to lower and remove punctuation
        user_input = re.sub(r'[^\w\s]', '', user_input.lower())

        for word in keywords:
            if word in user_input:
                self.get_logger().info('Word found: "%s"' % word)
                return True, word
        
        return False, None
    
    def say_bye(self):
        self.speak(f'Nice chatting with you. Have a nice day!')
        self.want_snacks = False
        self.bye_found = True

    def find_bye(self, user_input):
        bye_found, _ = self.look_for_keywords(user_input, self.exit_cmd_options)
        if bye_found:
            self.say_bye()
            return True
        return False
        
    def get_snack_wanted(self):
        """ Asks user for which snack is desired, looks for key snacks in the user input
        and returns the 

        Returns:
            snack_wanted_found (bool): True if it could find valid snack keyword in the user_input
        """
 
        while self.want_snacks: #and self.active:
            self.speak('What would you like to have?')
            self.speak(f'Your options are{self.snack_options_str}.')
            self.speak('You can say "bye" if you have changed your mind.')
            user_input = self.get_input()

            # Repeat question if user does not say anything
            if user_input is None:
                continue
            
            found, snack_wanted = self.look_for_keywords(user_input, self.snack_options)

            if found:
                self.snack_wanted = snack_wanted
                return True
            elif self.find_bye(user_input):
                return False
    
    def chat_with_user(self, user_input):
        while user_input is not None:
            self.messages.append({"role" : "user", "content" : user_input})

            # Print available openai models with:
            # print(openai.Model.list())
            completion = openai.ChatCompletion.create(
                    model= "gpt-3.5-turbo",
                    messages=self.messages,
                    temperature=0.8
                )      

            response = completion.choices[0].message.content
            self.messages.append({"role": "assistant", "content": response})
            print(f"\n{response}\n")
            self.save_inprogress()
            
            self.speak(f'{response}')
            
            user_input = self.get_input()

            if self.find_bye(user_input):
                break
        
        self.say_bye()

    def run(self):
        if self.active:
            # Introduce upon reaching the human
            intro = ['Hi! Would you like to have snacks?',
                    f'I have{self.snack_options_str}. What would you like?',
                    'If you are not hungry, you can ask me anything.']
            
            self.speak(intro)

            # Get user prompt
            user_input = self.get_input()

            # Repeat intro if user did not say anything
            if user_input is None:
                return
            
            self.want_snacks, _ = self.look_for_keywords(user_input, ['snacks'] + self.snack_options)

            if self.want_snacks:
                if self.get_snack_wanted():
                    self.request_snacks()
            else:
                self.chat_with_user(user_input)
            
            time.sleep(5)
            self.bye_found = False



def main(args=None):
    rclpy.init(args=args)

    voice_assistant_node = VoiceAssistant()

    while rclpy.ok():
        rclpy.spin_once(voice_assistant_node)
        voice_assistant_node.run()
        
    
    voice_assistant_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    




