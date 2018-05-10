#!/usr/bin/env python
# coding=utf-8
#from __future__ import unicode_literals
import requests
import sys
import os



from rasa_nlu.training_data import load_data
from rasa_nlu.model import Trainer
from rasa_nlu import config
from rasa_nlu.model import Interpreter
import json



class RasaNLU():

    def __init__(self):

        # Dict containing the entity and their function
        self.intent_functions = {
            'guide_to': self.guide_to,
            'say_something': self.say_something,
            'count_object': self.count_object,
            'find_object': self.find_object,
            'follow': self.follow,
            'find_people': self.find_people,
            'deliver': self.deliver
        }

        sys.stderr = open(os.devnull, 'w')

        # Load the config files
        print("Loading config files...")

        training_data = load_data("robocup_gpsr.json")
        trainer = Trainer(config.load("config_spacy.yml"))

        print("Training the model...")
        # Train the model based on the robocup_spr.json file
        trainer.train(training_data)
        # Returns the directory the model is stored in
        model_directory = trainer.persist('default/')

        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("RasaNLU init done.")

    # args :	Name + Beacon1 + Beacon2
    def guide_to(self, f_arg=None):
        print('entity :', 'guide_to')

        object=""
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        if len(f_arg) == 3:
            if f_arg[0].get('entity') == 'object':
                print(": Navigate to the " + f_arg[0].get('value'))
                print(": Look for someone")
                print(": Ask if his name is " + f_arg[1].get('value'))
                print(": Guide the operator to the " + f_arg[2].get('value'))
                print(": Stop when the destination is reached")
            if f_arg[0].get('entity') == 'name':
                print(": Navigate to the " + f_arg[1].get('value'))
                print(": Look for someone")
                print(": Ask if his name is " + f_arg[0].get('value'))
                print(": Guide the operator to the " + f_arg[2].get('value'))
                print(": Stop when the destination is reached")

    # args: Name + Beacon
    #       Name + Room
    #       Beacon
    #       Room
    # $whattosay = something about yourself
    # $whattosay = the time
    # $whattosay = what day is (today | tomorrow)
    # $whattosay = your team's (name | country | affiliation)
    # $whattosay = the day of the(week | month)
    # $whattosay = a joke

    def say_something(self, f_arg=None):
        print('entity :', 'say_something')

        object=""
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

    # args :	Object + Placement
    def count_object(self, f_arg=None):
        print('entity :', 'count_object')

        object=""
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        if len(f_arg) == 2:
            print(": Navigate to the " + f_arg[1].get('value'))
            print(": Look for " + f_arg[0].get('value'))
            print(": Count them")
            print(": Navigate to operator")
            print(": Tell how many " + f_arg[0].get('value') + " there was")

    # args :	object + room
    def find_object(self, f_arg=None):
        print('entity :', 'find_object')

        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        if len(f_arg) == 2:
            print(": Navigate to the " + f_arg[1].get('value'))
            print(": Look for " + f_arg[0].get('value'))
            print(": Grasp it")
            print(": Navigate to operator")
            print(": Hand the " + f_arg[0].get('value') + " to the operator")


    # args:  name + beacon + room
    #        beacon + name
    def follow(self, f_arg=None):
        print('entity :', 'follow')

        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))
        print('*' * 40)

        if len(f_arg) == 3:
            print(": Navigate to the " + f_arg[1].get('value'))
            print(": Look for someone")
            print(": Ask if his name is " + f_arg[0].get('value'))
            print(": Follow him/her")
            print(": Stop when you reach the " + f_arg[2].get('value'))

        if len(f_arg) == 2:
            print(": Navigate to the " + f_arg[0].get('value'))
            print(": Look for someone")
            print(": Ask if his name is " + f_arg[1].get('value'))
            print(": Follow him/her")
            print(": Stop when he/she tells you")

    # args: beacon
    #       room
    def find_people(self, f_arg=None):
        print('entity :', 'find_people')

        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))
        print('*'*40)

        if len(f_arg) == 1:
            if f_arg[0].get('entity') == 'room':
                print(": Navigate to the " + f_arg[0].get('value'))
                print(": Look for someone")
                print(": Ask his name")
                print(": Navigate to operator")
                print(": Tell the name")

            elif f_arg[0].get('entity') == 'object':
                print(": Navigate to the " + f_arg[0].get('value'))
                print(": Look for someone")
                print(": Ask his name")
                print(": Navigate to operator")
                print(": Tell the name")

    # args:  name + beacon + object + placement
    #        object + placement1
    #        object + placement1 + name + beacon
    #        object + placement1 + placement2
    def deliver(self, f_arg=None):
        print('entity :', 'deliver')

        object=""
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

class Question():
    def __init__(self):
        self.RecoString = []
        self.ANSWERS = []
        self.QUESTIONS = dict()
        self.WEIGHT = dict()
        self.rasa = RasaNLU()
        print("Question init done.")

    def call_rasa(self, question):

        # return the JSON as a dict
        print("Calling rasa_nlu...")
        print('*' * 40)
        response = self.rasa.interpreter.parse(question)

        entities = response.get('entities')

        sentence = self.rasa.intent_functions[response.get('intent').get('name')](entities)

        return(sentence)

    def handle_answer_question(self, question):
        print("start wm_nlu")

        self.call_rasa(question)
        print('*' * 40)
        print(question)


if __name__ == "__main__":
    q = Question()
    question = "Meet Jane at the washbasin and guide her to the shower"
    q.handle_answer_question(question)


    ###############################
    # QUESTIONS EXAMPLES
    ###############################

