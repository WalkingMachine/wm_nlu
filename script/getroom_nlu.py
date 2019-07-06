#!/usr/bin/env python
# coding=utf-8
#from __future__ import unicode_literals
import requests
import sys
import os

import rospy
from std_msgs.msg import String

from rasa_nlu.training_data import load_data
from rasa_nlu.model import Trainer
from rasa_nlu import config
from rasa_nlu.model import Interpreter

import json
import xml.etree.ElementTree as ET

from sara_msgs.msg import *
from wm_nlu.srv import *
import rospkg


class RasaNLU():

    def __init__(self):

        # Dict containing the entity and their function
        self.intent_functions = {
            'get_info_point': self.get_room,
            'goodbye': self.goodbye
        }
        sys.stderr = open(os.devnull, 'w')
        
        # Dict containing the rooms and their synonyms
        self.rooms = {
            'bedroom': ["bedroom", "bed", "bed room"],
            'kitchen': ["diningroom", "dining room", "dining", "dinette", "eating", "eating place", "kitchen", "cooking area", "cooking"],
            'office': ["office", "work station", "workstation", "studio", "workspace", "workroom", "work", "study", "bureau", "desk"],
            'living room': ["living room", "living", "livingroom", "salon", "family room", "family", "lounge", "sitting room", "sitting", "parlor", "parlour"]
        }
        
        # Load the config files
        print("Loading config files...")
        rospack = rospkg.RosPack()
        training_data = load_data(rospack.get_path('wm_nlu')+"/script/robocupPH_getRoom.json")
        trainer = Trainer(config.load(rospack.get_path('wm_nlu')+"/script/config_spacy.yml"))

        print("Training the model...")
        # Train the model based on the robocupPH_getRoom.json file
        trainer.train(training_data)
        # Returns the directory the model is stored in
        model_directory = trainer.persist(rospack.get_path('wm_nlu')+'/script/default_robocupPH_getRoom/')
        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("RasaNLU init done.")

    # ARGS
    # -object
    # RETURN : object
    # Will try to get an object name with synonyms
    def get_room(self, f_arg=None):
        try:
            for room in self.rooms:
                for synonym in self.rooms[room]:
                    print("Comparing "+synonym+" and "+str(f_arg[0].get('value')))
                    if synonym == str(f_arg[0].get('value')):
                        # Detects and returns the key instead of the value if it is a synonym
                        print("Detected room: "+room)
                        return room
            return "none"
        except:
            return "none"

    def goodbye(self, f_arg=None):
        return "none"


class getRoomClass():
    def __init__(self):
        self.RecoString = []
        self.ANSWERS = []
        self.QUESTIONS = dict()
        self.WEIGHT = dict()
        self.rasa = RasaNLU()

    def call_rasa(self, question):

        # return the JSON as a dict
        print("Calling rasa_nlu...")
        print('*' * 40)
        response = self.rasa.interpreter.parse(question)
        entities = response.get('entities')
        print(response) #Returns no entities
        sentence = self.rasa.intent_functions[response.get('intent').get('name')](entities)
        return sentence

    def handle_get_room(self, req):
        print("start wm_nlu")
        print("received : "+str(req.str.data))
        try:
            raw_sentence = str(req.str.data)
            answer = self.call_rasa(raw_sentence.decode('utf-8'))
            if len(answer) > 0:
                return PHGetRoomResponse(String(answer))
            else:
                return PHGetRoomResponse(String("none"))
        except:
            return PHGetRoomResponse(String("except"))

    def get_room_server(self):
        rospy.init_node('get_room_server')
        s = rospy.Service('get_room', PHGetRoom, self.handle_get_room)
        print("Ready to get the room from speech.")
        rospy.spin()


if __name__ == "__main__":
    q = getRoomClass()
    q.get_room_server()


