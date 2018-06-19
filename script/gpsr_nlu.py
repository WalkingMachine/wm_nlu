#!/usr/bin/env python
# coding=utf-8
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
import difflib
import commands

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
        rospack = rospkg.RosPack()
        training_data = load_data(rospack.get_path('wm_nlu')+"/script/robocup_gpsr.json")
        trainer = Trainer(config.load(rospack.get_path('wm_nlu')+"/script/config_spacy.yml"))

        print("Training the model...")
        # Train the model based on the robocup_spr.json file
        trainer.train(training_data)
        # Returns the directory the model is stored in
        model_directory = trainer.persist(rospack.get_path('wm_nlu')+'/script/default_gpsr/')

        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("GPSR_NLU init done.")

    # createActionNavigate
    # Move to robot to the specified object or room
    def createActionNavigate(self, arg):
        action = ActionMsg()
        action.Action = "move"
        action.args.append(arg)

        print(": Navigate to the " + arg)
        return action

    # createActionFind
    # Move to robot to the specified object or room
    #   arg : string object
    #         string place
    def createActionFind(self, arg):
        action = ActionMsg()
        action.Action = "Find"
        action.args.append(arg) # what to find
        #action.args.append("")  # where to look for, empty = actual place

        print(": Look for a " + arg)
        return action

    # createActionFindPerson
    # Find the operator by his name
    #   arg : string name
    def createActionFindPerson(self, arg=None):
        action = ActionMsg()
        action.Action = "FindPerson"

        if arg != None:
            action.args.append(arg)  # operator name
            print(": Find " + arg + " in the room")
        else:
            print(": Find operator in the room")

        return action

    # createActionGuidePerson
    # Guide operator to room
    #   arg : string room
    def createActionGuidePerson(self, arg):
        action = ActionMsg()
        action.Action = "Guide"

        action.args.append(arg)  # room name

        print(": Guide the operator to the " + arg)
        return action

    # createActionPickObject
    # Pick-up an object
    #   arg : string objectName
    def createActionPickObject(self, arg):
        action = ActionMsg()
        action.Action = "Pick"

        action.args.append(arg)  # object name

        print(": Pick the " + arg)
        return action

    # createActionGive
    # Give the content of the gripper to a person
    #   arg : string personName
    def createActionGive(self, arg=None):
        action = ActionMsg()
        action.Action = "Give"

        if arg != None:
            action.args.append(arg)  # person name
            print(": Give the object to " + arg)
        else:
            print(": Give the object to the operator")
        return action

    # createActionSay
    # Say something
    #   arg : string sentence
    def createActionSay(self, arg):
        action = ActionMsg()
        action.Action = "Say"

        action.args.append(arg)  # sentence to say

        print(": Say : " + arg)
        return action

    # createActionAsk
    # Ask something
    #   arg : string question
    def createActionAsk(self, arg, arg1):
        action = ActionMsg()
        action.Action = "Ask"

        action.args.append(arg)  # question to ask
        action.args.append(arg1)  # Where to store the answer

        print(": Ask : " + arg)
        return action

    # createActionFollow
    # Follow someone
    #   arg : string person_name
    def createActionFollow(self, arg):
        action = ActionMsg()
        action.Action = "Follow"

        action.args.append(arg)  # question to ask

        print(": Follow " + arg)
        return action

    # createActionCount
    # count objects
    #   arg : string objects_name
    #   arg : string value_name
    def createActionCount(self, arg, value):
        action = ActionMsg()
        action.Action = "Count"

        action.args.append(arg) # object to count
        action.args.append(value) # Rosparam

        print(": Count " + arg)
        return action

    # createActionPlace
    # place the content of the gripper on something
    #   arg : string Location
    def createActionPlace(self,arg):
        action = ActionMsg()
        action.Action = "Place"

        action.args.append(arg) # object to place on

        print(": Place on " + arg)
        return action


    # createActionAnswer
    # Answer a question
    def createActionAnswer(self,arg=None):
        action = ActionMsg()
        action.Action = "Answer"

        #action.args.append() # object to place on

        print(": Answer question ")
        return action

    # args :	Name + Beacon1 + Beacon2
    def guide_to(self, f_arg=None):
        print('entity :', 'guide_to')

        object=""
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        if len(f_arg) == 3:
            if acorrect(f_arg[0].get('entity')) == 'object':

                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will go near the " + acorrect(f_arg[0].get('value')) + " find " + f_arg[1].get('value') + " and guide him to the " + acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
                #msg.actions.append(self.createActionFind('person'))
                msg.actions.append(self.createActionFindPerson(f_arg[1].get('value')))
                msg.actions.append(self.createActionGuidePerson(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate('operator'))

                return msg


            if acorrect(f_arg[0].get('entity')) == 'name':

                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[1].get('value')) + " find " + f_arg[0].get('value') + " and guide him to the " + acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
                #msg.actions.append(self.createActionFind('person'))
                msg.actions.append(self.createActionFindPerson(f_arg[0].get('value')))
                msg.actions.append(self.createActionGuidePerson(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate('operator'))

                return msg

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
            return "I'd like to tell you something, " + self.randomString(yourself)
        elif "time" in words:
            return "It's currently " + str(datetime.now().strftime("%H:%M:%S"))
        elif "today" in words:
            return "Today we are " + datetime.now().strftime("%B %d")
        elif "tomorrow" in words:
            d = datetime.today()+timedelta(days=1)
            return "Tomorrow we are " + d.strftime("%B %d")
        elif "name" in words:
            return "My team's name is Walking Machine"
        elif "country" in words:
            return "We are from Montreal, Canada"
        elif "affiliation" in words:
            return "We are from Ecole de Technologie Superieure"
        elif "joke" in words:
            return "I'd like to tell you a joke. " + self.randomString(joke)
        return "SPR"

    def say_something(self, f_arg=None):
        print('entity :', 'say_something')

        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        sentenceToSay = str(self.whatToSay(self.sentence))
        if sentenceToSay == "SPR":
            self.answerAction = self.createActionAnswer()
        else:
            self.answerAction = self.createActionSay(sentenceToSay)

        if len(f_arg) == 1:
            msg = ActionArrayMsg()

            msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[0].get('value')) + " find a person, answer a question and come back here"))
            msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
            msg.actions.append(self.createActionFindPerson())
            msg.actions.append(self.answerAction)
            msg.actions.append(self.createActionNavigate('operator'))

            return msg

        elif len(f_arg) >= 2:
            if acorrect(f_arg[0].get('entity')) == 'name':
                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[1].get('value')) + " find "+f_arg[0].get('value')+", answer a question and come back here"))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionFindPerson(f_arg[0].get('value')))
                msg.actions.append(self.answerAction)
                msg.actions.append(self.createActionNavigate('operator'))

                return msg


            if acorrect(f_arg[0].get('entity')) == 'room':

                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[0].get('value')) + " find " + f_arg[1].get('value') + ", answer a question and come back here"))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionFindPerson(f_arg[1].get('value')))
                msg.actions.append(self.answerAction)
                msg.actions.append(self.createActionNavigate('operator'))

                return msg

    # args :	Object + Placement
    def count_object(self, f_arg=None):
        print('entity :', 'count_object')

        object=""
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        if len(f_arg) == 2:
            msg = ActionArrayMsg()

            msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[1].get('value')) + " to count the number of " + acorrect(f_arg[0].get('value')) + " and come back here to give you the answer"))
            msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
            #msg.actions.append(self.createActionFind(f_arg[0].get('value')))
            msg.actions.append(self.createActionCount(acorrect(f_arg[0].get('value')), 'behavior/Count/NbObjects'))
            msg.actions.append(self.createActionNavigate('operator'))
            msg.actions.append(self.createActionSay("There's $behavior/Count/NbObjects " + acorrect(f_arg[0].get('value'))))

            return msg

    # args :	object + room
    def find_object(self, f_arg=None):
        print('entity :', 'find_object')

        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        print('*' * 40)

        if len(f_arg) == 2:

            msg = ActionArrayMsg()

            msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[1].get('value')) + " to pick the " + acorrect(f_arg[0].get('value')) + " and come back here to give it to you"))
            msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
            msg.actions.append(self.createActionFind(acorrect(f_arg[0].get('value'))))
            msg.actions.append(self.createActionPickObject(acorrect(f_arg[0].get('value'))))
            msg.actions.append(self.createActionNavigate('operator'))
            msg.actions.append(self.createActionGive())

            return msg

    # args:  name + beacon + room
    #        beacon + name
    def follow(self, f_arg=None):
        print('entity :', 'follow')

        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))
        print('*' * 40)

        if len(f_arg) == 3:

            msg = ActionArrayMsg()

            msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[1].get('value')) + " to find " + f_arg[0].get('value') + " and follow him"))
            msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
            msg.actions.append(self.createActionFindPerson(f_arg[0].get('value')))
            msg.actions.append(self.createActionFollow(f_arg[1].get('value')))
            msg.actions.append(self.createActionNavigate('operator'))

            return msg

        if len(f_arg) == 2:

            msg = ActionArrayMsg()

            msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[0].get('value')) + " to find " + f_arg[1].get('value') + " and follow him"))
            msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
            msg.actions.append(self.createActionFindPerson(f_arg[1].get('value')))
            msg.actions.append(self.createActionFollow(f_arg[1].get('value')))
            msg.actions.append(self.createActionNavigate('operator'))

            return msg

    # args: beacon
    #       room
    def find_people(self, f_arg=None):
        print('entity :', 'find_people')

        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))
        print('*'*40)

        if len(f_arg) == 1:
            if acorrect(f_arg[0].get('entity')) == 'room':
                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will go to the " + acorrect(f_arg[0].get('value')) + " to ask the name of the person"))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionFindPerson())
                msg.actions.append(self.createActionAsk('What is your name', 'behavior/Answer/Name'))
                msg.actions.append(self.createActionNavigate('operator'))
                msg.actions.append(self.createActionSay('The name of the person is $behavior/Answer/Name.'))

                return msg


            elif acorrect(f_arg[0].get('entity')) == 'object':
                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will ask the name of the person near the " + acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionFindPerson())
                msg.actions.append(self.createActionAsk('What is your name', 'behavior/Answer/Name'))
                msg.actions.append(self.createActionNavigate('operator'))
                msg.actions.append(self.createActionSay('The name of the person is $behavior/Answer/Name.'))

                return msg


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

        if len(f_arg) == 1:

            msg = ActionArrayMsg()

            msg.actions.append(self.createActionSay("I will pick the " + acorrect(f_arg[0].get('value')) + " and bring it here"))
            msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
            msg.actions.append(self.createActionFind(acorrect(f_arg[0].get('value'))))
            msg.actions.append(self.createActionPickObject(acorrect(f_arg[0].get('value'))))
            msg.actions.append(self.createActionNavigate('operator'))
            msg.actions.append(self.createActionGive())

            return msg

        if len(f_arg) == 2:

            if "go" in self.sentence.split() or "navigate" in self.sentence.split():
                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay(
                    "I will pick the " + acorrect(f_arg[1].get('value')) + " from the " + acorrect(f_arg[0].get(
                        'value')) + " and bring it here"))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionFind(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionPickObject(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionNavigate('operator'))
                msg.actions.append(self.createActionGive())

                return msg

            elif "place" in self.sentence.split() or "put" in self.sentence.split():
                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will pick the " + acorrect(f_arg[0].get('value')) + " and place it on the " + acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionFind(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionPickObject(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionPlace(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionNavigate('operator'))

                return msg

            else:
                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will pick the " + acorrect(f_arg[0].get('value')) + " from the "+acorrect(f_arg[1].get('value'))+" and bring it here"))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionFind(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionPickObject(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionNavigate('operator'))
                msg.actions.append(self.createActionGive())

                return msg

        elif len(f_arg) == 3:

            if "go" in self.sentence.split() or "navigate" in self.sentence.split():

                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will pick the " + acorrect(f_arg[1].get('value')) + " from the " + acorrect(f_arg[0].get('value')) + " and place it on the "+acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionFind(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionPickObject(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionPlace(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate('operator'))

                return msg

            elif "on" in self.sentence.split() or "from" in self.sentence.split():

                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will pick the " + acorrect(f_arg[0].get('value')) + " from the " + acorrect(f_arg[1].get('value')) + " and place it on the " + acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionFind(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionPickObject(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionPlace(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate('operator'))

                return msg

            else:

                msg = ActionArrayMsg()


                msg.actions.append(self.createActionSay("I will pick the " + acorrect(f_arg[1].get('value')) + " in the " + acorrect(f_arg[2].get('value')) + " and give it to " + f_arg[0].get('value')))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionFind(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionPickObject(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionFindPerson(f_arg[0].get('value')))
                msg.actions.append(self.createActionGive())
                msg.actions.append(self.createActionNavigate('operator'))

                return msg

        elif len(f_arg) == 4:
            if f_arg[0].get('entity') == 'name':

                msg = ActionArrayMsg()

                msg.actions.append(self.createActionSay("I will pick the " + acorrect(f_arg[2].get('value')) + " from the " + acorrect(f_arg[3].get('value')) + ", move to the " + acorrect(f_arg[1].get('value'))+ " and give it to " +f_arg[0].get('value')))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[3].get('value'))))
                msg.actions.append(self.createActionFind(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionPickObject(acorrect(f_arg[2].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionFindPerson(f_arg[0].get('value')))
                msg.actions.append(self.createActionGive())
                msg.actions.append(self.createActionNavigate('operator'))

                return msg
 
            elif f_arg[2].get('entity') == 'name':

                msg = ActionArrayMsg()


                msg.actions.append(self.createActionSay("I will pick the " + acorrect(f_arg[0].get('value')) + " from the " + acorrect(f_arg[1].get('value')) + ", move to the " + acorrect(f_arg[3].get('value'))+ " and give it to " +f_arg[2].get('value')))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[1].get('value'))))
                msg.actions.append(self.createActionFind(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionPickObject(acorrect(f_arg[0].get('value'))))
                msg.actions.append(self.createActionNavigate(acorrect(f_arg[3].get('value'))))
                msg.actions.append(self.createActionFindPerson(f_arg[2].get('value')))
                msg.actions.append(self.createActionGive())
                msg.actions.append(self.createActionNavigate('operator'))

                return msg

    def acorrect(heardWord):
        phonemesHeard==commands.getoutput("espeak -x "+heardWord)
        possibleWords=["bag","cloth","scrubby","sponge","basket","tray","chocolate drink","coke","grape juice","orange juice","sprite","cereal","noodles",
                "sausages", "apple","orange","paprika","crackers","potato chips","pringles"   ,   "additional","cleaning stuff","drinks","food","snacks",
                "entrance","corridor","kitchen","storage table","sink","dishwasher","counter","ng room","dining table","side table","bedroom","bed","desk",
                "living room","end table","couch","bookcase"]
        
        phonemesToWord={}
        phonemesPossible=[]
        for x in possibleWords:
                phonemesPossible.append(commands.getoutput("espeak -x -q "+x))
                phonemesToWord[phonemesPossible[-1]]=x
        realPhoneme=difflib.get_close_matches(phonemesHeard,phonemesPossible)
        return phonemesToWord[realPhoneme[0]]

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

    def handle_get_actions(self, req):

        order = str(req.order.data)

        print("Received question : " + order)

        response = self.call_rasa(order.decode('utf-8'))
        print('*' * 40)
        print('')

        return GPSRReceiveActionResponse(response)

    def gpsr_nlu_server(self):
        rospy.init_node('gpsr_nlu_server')
        s = rospy.Service('gpsr_receive_action', GPSRReceiveAction, self.handle_get_actions)
        print "Ready to receive order."
        rospy.spin()


if __name__ == "__main__":
    o = Order()
    o.gpsr_nlu_server()


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
