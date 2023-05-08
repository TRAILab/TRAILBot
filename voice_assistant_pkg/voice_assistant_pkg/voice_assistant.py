import sys

from servo_interfaces.srv import RunServo
import rclpy
from rclpy.node import Node

import openai
import speech_recognition as sr
import pyttsx3
import os 
import json
import re
import time

personality = "You are a helpful assistant."
usewhisper = True # If false, use Google to transcribe your speech, else use openai transcriber
key = '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'

# openAI set-up
openai.api_key = key
messages  = [{"role": "system", "content": f"{personality}"}]

# pyttsx3 setup: text to speech voice generator
engine = pyttsx3.init()
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[11].id) # 0 for male, 1 for female
# for i, voice in enumerate(voices):
#     if voice.id == 'english':
#         print(i, voice)

# speech recognition set-up
r = sr.Recognizer()
# print(sr.Microphone.list_microphone_names())
mic = sr.Microphone(device_index=12) #11 or 12: self microphone, 13: snowball
r.dynamic_energy_threshold=False
r.energy_threshold = 200

def get_input():
    with mic as source:
        print("\nAmbient noise adjust...")
        r.adjust_for_ambient_noise(source)
        print("\nListening...")
        audio = r.listen(source)
        try:
            if usewhisper:
                user_input = whisper(audio)
                print('From whisper: ', user_input)
            else:
                user_input = r.recognize_google(audio)
                print('Transcribed from google: ', user_input)
            return user_input
        except:
            return ''


def whisper(audio):
    with open('speech.wav','wb') as f:
        f.write(audio.get_wav_data())
    speech = open('speech.wav', 'rb')
    wcompletion = openai.Audio.transcribe(
        model = "whisper-1",
        file=speech
    )
    user_input = wcompletion['text']
    return user_input


def save_conversation(save_foldername):
    '''
    Checks the folder for previous conversations and will get the next suffix that has not been used yet.  
    It returns suffix number

    Args:
        save_foldername (str) : Takes in the path to save the conversation to.
    '''
    
    os.makedirs(save_foldername, exist_ok=True)

    base_filename = 'conversation'
    suffix = 0
    filename = os.path.join(save_foldername, f'{base_filename}_{suffix}.txt')

    while os.path.exists(filename):
        suffix += 1
        filename = os.path.join(save_foldername, f'{base_filename}_{suffix}.txt')

    with open(filename, 'w') as file:
        json.dump(messages, file, indent=4)

    return suffix

def save_inprogress(suffix, save_foldername):
    '''
    Uses the suffix number returned from save_conversation to continually update the 
    file for this instance of execution.  This is so that you can save the conversation 
    as you go so if it crashes, you don't lose to conversation.  Shouldn't be called
    from outside of the class.

    Args:
        suffix  :  Takes suffix count from save_conversation()
    '''
    os.makedirs(save_foldername, exist_ok=True)
    base_filename = 'conversation'
    filename = os.path.join(save_foldername, f'{base_filename}_{suffix}.txt')

    with open(filename, 'w') as file:
        json.dump(messages, file, indent=4)

class RunServoClient(Node):

    def __init__(self):
        super().__init__('run_servo_client_node')
        self.cli = self.create_client(RunServo, 'runservo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RunServo.Request()

    def send_request(self, servo):
        self.req.servo = servo
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    run_servo_client_node = RunServoClient()

    # grab script location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    foldername = "voice_assistant"
    save_foldername = os.path.join(script_dir,f"conversations/{foldername}")
    suffix = save_conversation(save_foldername)

    # main while loop where the conversation occurs
    while rclpy.ok():
        # TODO: Subscribe to a topic that indicates human is reached

        # Introduce upon reaching the human
        engine.say(f'Hello pretty lady! Would you like to have snacks or chat with me?')
        engine.say(f'Say "snacks", if you would like to have snacks.')
        engine.say(f'Otherwise, you can ask me any question.')
        engine.runAndWait()

        # Get user prompt
        user_input = get_input()
        if user_input == '':
            continue
        
        # Look for the word 'snacks' or similar in user's prompt
        want_snacks = False
        snack_wanted = 0

        for word in user_input.split():
            word = re.sub(r'[^\w\s]', '', word.lower())
            if word in ['snacks', 'chocolate', 'chips', 'nuts', 'candy']:
                want_snacks = True
                break
        
        if want_snacks:
            # Find which snack to dispense
            while True:
                engine.say(f'What would you like to have? Your options are 1, 2, 3 or 4')
                engine.runAndWait()
                user_input = get_input()
                if user_input == '':
                    continue
                elif snack_wanted == 0:
                    for word in user_input.split():
                        word = re.sub(r'[^\w\s]', '', word.lower())
                        if word in ['one', '1']:
                            snack_wanted = 1
                        elif word in ['two', '2']:
                            snack_wanted = 2
                        elif word in ['three', '3']:
                            snack_wanted = 3
                        elif word in ['four', '4']:
                            snack_wanted = 4
                    if snack_wanted > 0: 
                        break
                    else:
                        continue # Ask again

        else:
            # Enable chatGPT mode
            end_chat = False
            while rclpy.ok() and not end_chat and user_input != '':
                # user_input = input('What question do you want to ask: ')
                messages.append({"role" : "user", "content" : user_input})

                completion = openai.ChatCompletion.create(
                        model= "gpt-3.5-turbo",
                        messages=messages,
                        temperature=0.8
                    )      # print(openai.Model.list())

                response = completion.choices[0].message.content
                messages.append({"role": "assistant", "content": response})
                print(f"\n{response}\n")
                save_inprogress(suffix, save_foldername)
                
                engine.say(f'{response}')
                engine.runAndWait()
                
                user_input = get_input()
                
                # Look for word that prompts to end conversation 
                for word in user_input.split():
                    word = re.sub(r'[^\w\s]', '', word.lower())
                    if word in ['bye', 'bubye', 'adios', 'ciao', 'thanks', 'thank']:
                        end_chat = True
                        engine.say(f'Nice chatting with you, see you!')
                        engine.runAndWait()
                        break

        # Dispense Snacks
        if want_snacks and snack_wanted>0:
            engine.say(f'Alrighty, Snack number {snack_wanted} coming right up!')
            engine.runAndWait()
            
            # Run Servo and Dispense Snack
            response = run_servo_client_node.send_request(int(snack_wanted))
            # Print Response
            run_servo_client_node.get_logger().info(f'Response of running servo: {int(snack_wanted)} -> {response.success}, {response.message}')

            if response.success:
                engine.say(f'Please pick up your snack from my side!')
                engine.say(f'Thank you! Have a nice Day!')
            else:
                engine.say(f'Oops, there seems to be a problem!')
            engine.runAndWait()
        
        time.sleep(5)
    
    run_servo_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    




