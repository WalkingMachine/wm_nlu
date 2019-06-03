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
            'order': self.order
        }
        self.sentence = ""

        # Load the config files
        print("Loading config files...")
        rospack = rospkg.RosPack()
        training_data = load_data(rospack.get_path('wm_nlu')+"/script/test_restaurant.json")
        trainer = Trainer(config.load(rospack.get_path('wm_nlu')+"/script/config_spacy.yml"))

        print("Training the model...")
        # Train the model based on the robocup_spr.json file
        trainer.train(training_data)
        # Returns the directory the model is stored in
        model_directory = trainer.persist(rospack.get_path('wm_nlu')+'/script/default_retaurant/')

        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("restaurant_NLU init done.")

    def order(self, f_arg=None):
        try:
            orderList = []
            for arg in f_arg:
                print(arg.get('entity'), ":", arg.get('value'))
                orderList.append(arg.get('value'))

            if len(orderList) == 0:
                print("unknown")
                return "unknown"
            else:
                return orderList
        except:
            print("unknown")
            return "unknown"



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

        sentence = str(req.str.data)

        print("Received sentence : " + sentence)

        response = self.call_rasa(sentence.decode('utf-8'))
        print('*' * 40)
        print('')
        msg = RestaurantNLUServiceResponse()
        for item in response:
            msg.order.append(String(str(item)))

        print("Response: "+ str(msg.order))
        return RestaurantNLUServiceResponse(msg.order)

    def restaurant_nlu_server(self):
        rospy.init_node('restaurant_nlu_server')
        s = rospy.Service('Restaurant_NLU_Service', RestaurantNLUService, self.handle_get_response)
        print "Ready to analyse."
        rospy.spin()


if __name__ == "__main__":
    o = Order()
    o.restaurant_nlu_server()
