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
            'get_room': self.get_room,
            'goodbye': self.goodbye
        }
        sys.stderr = open(os.devnull, 'w')

        # Load the config files
        print("Loading config files...")
        rospack = rospkg.RosPack()
        training_data = load_data(rospack.get_path('wm_nlu')+"/script/robocupHK_getRoom.json")
        #training_data = load_data(rospack.get_path('wm_nlu')+"/script/robocup_spr.json")
        trainer = Trainer(config.load(rospack.get_path('wm_nlu')+"/script/config_spacy.yml"))

        print("Training the model...")
        # Train the model based on the robocupHK_getRoom.json file
        trainer.train(training_data)
        # Returns the directory the model is stored in
        model_directory = trainer.persist(rospack.get_path('wm_nlu')+'/script/default_robocupHK_getRoom/')
        #model_directory = trainer.persist(rospack.get_path('wm_nlu')+'/script/default_robocup_spr/')
        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("RasaNLU init done.")

    # ARGS
    # -object
    # RETURN : room
    def get_room(self, f_arg=None):
	try:
	    print("Detected room: "+str(f_arg[0].get('value')))
	    return str(f_arg[0].get('value'))
	except:
	    return "none"

    def goodbye(self, f_arg=None):
	return "none"

class GetRoomClass():
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
        #print('*' * 40)
        entities = response.get('entities')
        #print(str(entities))
        #print('*' * 40)
        sentence = self.rasa.intent_functions[response.get('intent').get('name')](entities)
        #print("sentence")
        return(sentence)

    def handle_get_room(self, req):
        print("start wm_nlu")
	print("received : "+str(req.str.data))
        try:
            raw_sentence = str(req.str.data)
            answer = self.call_rasa(raw_sentence.decode('utf-8'))
            if len(answer) > 0:
                return HKGetRoomResponse(String(answer))
            else:
                return HKGetRoomResponse(String("none"))
        except:
            return HKGetRoomResponse(String("except"))

    def get_room_server(self):
        rospy.init_node('get_room_server')
        s = rospy.Service('get_room', HKGetRoom, self.handle_get_room)
        #s = rospy.Service('get_room', AnswerQuestion, self.handle_get_room)
        print("Ready to get the room.")
        rospy.spin()


if __name__ == "__main__":
    q = GetRoomClass()
    q.get_room_server()


