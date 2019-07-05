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
            'get_object': self.get_object,
            'goodbye': self.goodbye
        }
        sys.stderr = open(os.devnull, 'w')

       # Dict containing the rooms and their synonyms
        self.objects = {
            'Sink': [],
            'Display cabinet': [],
            'Kitchen cabinet': [],
            'Sideboard': [],
            'Desk': [],
            'Shelf': [],
            'Kitchen table': [],
            'bed': [],
            'side table': [],
            'bedroom chest': [],
            'shoe rack': [],
            'dishwasher': [],
            'fridge coat hanger': ["coat hanger"],
            'island': [],
            'trash': [],
            'bin': [],
            'tv': ["television"],
            'couch': ["sofa"],
            'coffee': [],
            'table': [],
            'armchair':["chair"],
            'plant': []
        }
	
        # Load the config files
        print("Loading config files...")
        rospack = rospkg.RosPack()
        training_data = load_data(rospack.get_path('wm_nlu')+"/script/robocupHK_getObject.json")
        trainer = Trainer(config.load(rospack.get_path('wm_nlu')+"/script/config_spacy.yml"))

        print("Training the model...")
        # Train the model based on the robocupPH_getRoom.json file
        trainer.train(training_data)
        # Returns the directory the model is stored in
        model_directory = trainer.persist(rospack.get_path('wm_nlu')+'/script/default_robocupHK_getObject/')
        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("RasaNLU init done.")

    # ARGS
    # -object
    # RETURN : object
    # Will try to get an object name with synonyms
    def get_object(self, f_arg=None):
        try:
            for object in self.objects.keys():
                for synonym in self.objects[object]:
                    print("Comparing "+synonym+" and "+str(f_arg[0].get('value')))
                    if synonym == str(f_arg[0].get('value')):
                        # Detects and returns the key instead of the value if it is a synonym
                        print("Detected object: "+object)
                        return object
            return "none"
        except:
            return "none"

    def goodbye(self, f_arg=None):
        return "none"

class GetObjectClass():
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
        sentence = self.rasa.intent_functions[response.get('intent').get('name')](entities)
        return(sentence)

    def handle_get_object(self, req):
        print("start wm_nlu")
        print("received : "+str(req.str.data))
        try:
            raw_sentence = str(req.str.data)
            answer = self.call_rasa(raw_sentence.decode('utf-8'))
            if len(answer) > 0:
                return PHGetObjectResponse(String(answer))
            else:
                return PHGetObjectResponse(String("none"))
        except:
            return PHGetObjectResponse(String("except"))

    def get_object_server(self):
        rospy.init_node('get_object_server')
        s = rospy.Service('get_object', PHGetObject, self.handle_get_object)
        print("Ready to get the object from speech.")
        rospy.spin()


if __name__ == "__main__":
    q = GetObjectClass()
    q.get_object_server()


