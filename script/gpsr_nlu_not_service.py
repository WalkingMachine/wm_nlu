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

import random

from datetime import datetime, timedelta

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
        self.sentence = ""
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

    # args: Name + Beacon       # TODO
    #       Name + Room         # TODO
    #       Beacon              # TODO
    #       Room                # TODO
    # $whattosay = something about yourself
    # $whattosay = the time
    # $whattosay = what day is (today | tomorrow)
    # $whattosay = your team's (name | country | affiliation)
    # $whattosay = the day of the(week | month)  #todo ? wtf
    # $whattosay = a joke

    def randomString(self, sentenceTable):
        return random.choice(sentenceTable)

    def whatToSay(self, sentence):
        joke = ["What is a robot’s favorite type of music? Heavy metal !",
                "What did the man say to his dead robot? Rust in peace.",
                "What do you get when you cross a robot and a tractor? A trans-farmer",
                "How many robots does it take to screw in a light bulb? Three, one to hold the bulb, and two to turn the ladder!"]
        yourself = ["I only have one arm",
                    "My name is Sara",
                    "I live in Montreal",
                    "I can speak 2 languages, english and french"]

        words = sentence.split()
        if "yourself" in words:
            return "Say : I'd like to tell you something, " + self.randomString(yourself)
        elif "time" in words:
            return "Say : It's currently " + str(datetime.now().strftime("%H:%M:%S"))
        elif "today" in words:
            return "Say : Today we are " + datetime.now().strftime("%B %d")
        elif "tomorrow" in words:
            d = datetime.today()+timedelta(days=1)
            return "Say : Tomorrow we are " + d.strftime("%B %d")
        elif "name" in words:
            return "Say : My team's name is Walking Machine"
        elif "country" in words:
            return "Say : We are from Montreal, Canada"
        elif "affiliation" in words:
            return "say : We are from Ecole de Technologie Superieure"
        elif "joke" in words:
            return "Say : I'd like to tell you a joke. " + self.randomString(joke)
        return "Answer question"

    def say_something(self, f_arg=None):
        print('entity :', 'say_something')

        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        if len(f_arg) == 1:
            print(": Navigate to the " + f_arg[0].get('value'))
            print(": Look for someone")
            print(": " + str(self.whatToSay(self.sentence)))
            print(": Navigate to operator")

        elif len(f_arg) >= 2:
            if f_arg[0].get('entity') == 'name':
                print(": Navigate to the " + f_arg[1].get('value'))
                print(": Look for " + f_arg[0].get('value'))
                print(": " + str(self.whatToSay(self.sentence)))
                print(": Navigate to operator")

            if f_arg[0].get('entity') == 'room':
                print(": Navigate to the " + f_arg[0].get('value'))
                print(": Look for " + f_arg[1].get('value'))
                print(": " + str(self.whatToSay(self.sentence)))
                print(": Navigate to operator")

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
    #        object + object + placement1
    #
    # Notes : Maybe create a new entity (put_on)
    #         to divide the sentence easily
    def deliver(self, f_arg=None):
        print('entity :', 'deliver')

        object=""
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        if len(f_arg) == 2:
            print(": Navigate to the " + f_arg[1].get('value'))
            print(": Look for the " + f_arg[0].get('value'))
            print(": Grasp it")
            print(": Navigate to the operator")
            print(": Hand over the " + f_arg[0].get('value') + " to the operator")

        elif len(f_arg) == 3:
            if "on" in self.sentence.split():
                print(": Navigate to the " + f_arg[1].get('value'))
                print(": Look for the " + f_arg[0].get('value'))
                print(": Grasp it")
                print(": Navigate back to the " + f_arg[2].get('value'))
                print(": Put the " + f_arg[0].get('value') + " on the " + f_arg[2].get('value'))
            else:
                print(": Navigate to the " + f_arg[2].get('value'))
                print(": Look for the " + f_arg[1].get('value'))
                print(": Grasp it")
                print(": Navigate back to the " + f_arg[0].get('value'))
                print(": Hand over the " + f_arg[1].get('value') + " to the operator")

        elif len(f_arg) == 4:
            if f_arg[0].get('entity') == 'name':
                print(": Navigate to the " + f_arg[3].get('value'))
                print(": Look for the " + f_arg[2].get('value'))
                print(": Grasp it")
                print(": Navigate back to the " + f_arg[1].get('value'))
                print(": Hand over the " + f_arg[2].get('value') + " to " + f_arg[0].get('value'))
            elif f_arg[2].get('entity') == 'name':
                print(": Navigate to the " + f_arg[1].get('value'))
                print(": Look for the " + f_arg[0].get('value'))
                print(": Grasp it")
                print(": Navigate back to the " + f_arg[3].get('value'))
                print(": Hand over the " + f_arg[0].get('value') + " to " + f_arg[2].get('value'))


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

    def handle_answer_question(self):
        print("start wm_nlu")

        self.call_rasa(self.rasa.sentence)
        print('*' * 40)
        print(self.rasa.sentence)


if __name__ == "__main__":
    q = Question()
    q.rasa.sentence = "Go to the bathroom, find Hayden, and tell your team’s name"
    q.handle_answer_question()


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
