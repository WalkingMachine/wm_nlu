#!/usr/bin/env python
# coding: utf-8
#from __future__ import unicode_literals
import requests
import sys
import requests
import os

import rospy
from std_msgs.msg import String
from wm_nlu.msg import *

from rasa_nlu.training_data import load_data
from rasa_nlu.model import Trainer
from rasa_nlu import config
from rasa_nlu.model import Interpreter

import json
import xml.etree.ElementTree as ET

import random

from datetime import datetime, timedelta

from sara_msgs.msg import *
from wm_nlu.srv import *
import rospkg

class RasaNLU():

    def __init__(self):

        # Dict containing the entity and their function
        self.intent_functions = {
            'favorite_drink': self.drink,
            'person_name': self.name
        }
        self.sentence = ""

        # Load the config files
        print("Loading config files...")
        rospack = rospkg.RosPack()
        training_data = load_data(rospack.get_path('wm_nlu')+"/script/test_receptionist.json")
        trainer = Trainer(config.load(rospack.get_path('wm_nlu')+"/script/config_spacy.yml"))

        print("Training the model...")
        # Train the model based on the robocup_spr.json file
        trainer.train(training_data)
        # Returns the directory the model is stored in
        model_directory = trainer.persist(rospack.get_path('wm_nlu')+'/script/default_receptionist/')

        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("receptionist_NLU init done.")

    def drink(self, f_arg=None):
        try:
            print("FAVORITE DRINK: "+ str(f_arg[0].get('value')))
            return f_arg[0].get('value')
        except:
            print("unknown")
            return "unknown"

    def name(self, f_arg=None):
        try:
            print("PERSON NAME: "+ str(f_arg[0].get('value')))
            return f_arg[0].get('value')
        except:
            print("unknown")
            return "unknown"


#    # createActionAnswer
#    # Answer a question
#    def createActionAnswer(self,arg=None):
#        action = ActionMsg()
#        action.Action = "Answer"

#action.args.append() # object to place on

#        print(": Answer question ")
#        return action

# args: beacon
#       room
#    def find_people(self, f_arg=None):
#        print('entity :', 'find_people')

#        for arg in f_arg:
#            print(arg.get('entity'), ":", arg.get('value'))
#        print('*'*40)

#        if len(f_arg) == 1:
#            if f_arg[0].get('entity') == 'room':
#                msg = ActionArrayMsg()

#                msg.actions.append(self.createActionSay("I will go to the " + f_arg[0].get('value') + " to ask the name of the person"))
#                msg.actions.append(self.createActionNavigate(f_arg[0].get('value')))
#                msg.actions.append(self.createActionFindPerson())
#                msg.actions.append(self.createActionAsk('What is your name', 'behavior/Answer/Name'))
#                msg.actions.append(self.createActionNavigate('operator'))
#                msg.actions.append(self.createActionSay('The name of the person is $behavior/Answer/Name.'))

#                return msg


#            elif f_arg[0].get('entity') == 'object':
#                msg = ActionArrayMsg()

#                msg.actions.append(self.createActionSay("I will ask the name of the person near the " + f_arg[0].get('value')))
#                msg.actions.append(self.createActionNavigate(f_arg[0].get('value')))
#                msg.actions.append(self.createActionFindPerson())
#                msg.actions.append(self.createActionAsk('What is your name', 'behavior/Answer/Name'))
#                msg.actions.append(self.createActionNavigate('operator'))
#                msg.actions.append(self.createActionSay('The name of the person is $behavior/Answer/Name.'))

#                return msg




class Order():
    def __init__(self):
        self.RecoString = []
        self.ANSWERS = []
        self.QUESTIONS = dict()
        self.WEIGHT = dict()
        self.rasa = RasaNLU()
        print("Question init done.")

    def call_rasa(self, question):
        self.rasa.sentence = question.lower()
        # return the JSON as a dict
        print("Calling rasa_nlu...")
        print('*' * 40)
        response = self.rasa.interpreter.parse(question)

        entities = response.get('entities')

        sentence = self.rasa.intent_functions[response.get('intent').get('name')](entities)

        return(sentence)

    def handle_get_response(self, req):

        sentence = str(req.str)

        print("Received sentence : " + sentence)

        if sentence.count(' ')<2:
            print("SHORT sentence")
            return ReceptionistNLUServiceResponse(sentence)

        response = self.call_rasa(sentence.decode('utf-8'))
        print('*' * 40)
        print('')

        return ReceptionistNLUServiceResponse(response)

    def receptionist_nlu_server(self):
        rospy.init_node('receptionist_nlu_server')
        s = rospy.Service('Receptionist_NLU_Service', ReceptionistNLUService, self.handle_get_response)
        print "Ready to analyse."
        rospy.spin()


if __name__ == "__main__":
    o = Order()
    o.receptionist_nlu_server()


    ###############################
    # QUESTIONS EXAMPLES
    ###############################
    # guide_to
    #  Guide Sophia from the bed to the dining table
    #  Meet Ryan at the  bathtub lead him to the stove
    #  Navigate to the sink, meet John, and accompany him to the coffe table
    #
    # say_something
    #  Locate Michael in the corridor and say what day is tomorrow
    #  Navigate to the living room, locate someone, and answer a question
    #
    # count_object
    #  Tell me how many bowl there are on the coffe table
    #
    # find_object
    #  Find the shampoo in the bathroom
    #
    # follow
    #  Come after Peyton from the stove to the dining room
    #  Meet Taylor at the bathroom cabinet and go after him to the kitchen
    #  Navigate to the sofa, meet Jo, and follow him
    #
    # find_people
    #  Tell me the name of the person at the bed
    #  Tell me the name of the person in the living room
    #
    # deliver
    #  Bring to Simon at the bed the box from the night table
    #  Grasp the fork from the microwave and give it to Jonathan at the baby chair
    #  Pick up the bowl from the dresser and put it on the bookshelf
    #
