import os
import json


class ConversationHandler:
    def __init__(self, messages, save_foldername):
        """AI is creating summary for __init__

        Args:
            messages (list of dicts): conversation 
            save_foldername (str) : Takes in the name of folder to save the conversation to
        """
        script_dir = os.path.dirname(
            os.path.abspath(__file__))  # grab script location
        self.save_foldername = os.path.join(script_dir, save_foldername)
        self.suffix = self.save_conversation(messages)

    def save_conversation(self, messages):
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
        filename = os.path.join(self.save_foldername,
                                f'{base_filename}_{suffix}.txt')

        while os.path.exists(filename):
            suffix += 1
            filename = os.path.join(
                self.save_foldername, f'{base_filename}_{suffix}.txt')

        with open(filename, 'w') as file:
            json.dump(messages, file, indent=4)

        return suffix

    def save_inprogress(self, messages):
        '''
        Uses the suffix number returned from save_conversation to continually update the 
        file for this instance of execution.  This is so that you can save the conversation 
        as you go so if it crashes, you don't lose to conversation.

        Args:
            messages (list of dicts): conversation
        '''
        os.makedirs(self.save_foldername, exist_ok=True)
        base_filename = 'conversation'
        filename = os.path.join(self.save_foldername,
                                f'{base_filename}_{self.suffix}.txt')

        with open(filename, 'w') as file:
            json.dump(messages, file, indent=4)
