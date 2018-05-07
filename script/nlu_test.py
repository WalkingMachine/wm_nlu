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


from wm_nlu.srv import *
import rospkg


# define state WaitingQuestion
class PredefinedQuestion():
    def __init__(self):
        self.RecoString = []
        self.ANSWERS = []
        self.QUESTIONS = dict()
        self.WEIGHT = dict()
        print("PredefinedQuestion init done.")

    def callback(self, question):
        self.RecoString = question.split()

        for word in self.RecoString:
            for question in self.QUESTIONS:
                if question.lower().find(word.lower()) != -1:
                    self.WEIGHT[question] += 1

    def loadQuestions(self, filename):
        print("Loading predefined question from", filename)
        tree = ET.parse(filename)
        root = tree.getroot()

        for childs in root:
            try:
                self.QUESTIONS[childs[0].text] = childs[1].text
                self.WEIGHT[childs[0].text] = 0

            except:
                print("error reading the file")
        print(self.QUESTIONS.__len__(), "predefined questions loaded")


class RasaNLU():

    def __init__(self):

        # Dict containing the entity and their function
        self.intent_functions = {
            'arena_where_located': self.arena_where_located,
            'arena_count_object': self.arena_count_object,
            'crowd_count': self.crowd_count,
            'crowd_person_gesture': self.crowd_person_gesture,
            'arena_color_object': self.arena_color_object,
            'arena_which_object': self.arena_which_object,
            'object_category': self.object_category,
            'object_adjective': self.object_adjective
        }

        sys.stderr = open(os.devnull, 'w')

        # Load the config files
        print("Loading config files...")
        rospack = rospkg.RosPack()
        training_data = load_data(rospack.get_path('wm_nlu')+"/script/robocup_spr.json")
        trainer = Trainer(config.load(rospack.get_path('wm_nlu')+"/script/config_spacy.yml"))

        print("Training the model...")
        # Train the model based on the robocup_spr.json file
        trainer.train(training_data)
        # Returns the directory the model is stored in
        model_directory = trainer.persist(rospack.get_path('wm_nlu')+'/script/default/')

        print("Loading the model...")
        self.interpreter = Interpreter.load(model_directory)
        print("RasaNLU init done.")

    def wonderland_get_first_entity(self, args = None ):

        url = "http://localhost:8000/api/entity/"

        headers = {
            'api-key': "43d5d4a6b88c4c2ea303188fcbc3385f79d1e961",
            'cache-control': "no-cache",
            'postman-token': "c0ef09c6-ad86-b7f4-52be-fe3c0f6d1238"
        }
        response = None

        if args != None:
            response = requests.get(url, headers=headers, params=args)
        else:
            response = requests.get(url, headers)

        parsedJson = json.loads(response.text)

        if isinstance(parsedJson, list) and len(parsedJson) > 0:
            return parsedJson[0]

        return parsedJson

    def wonderland_get_entity(self, args = None ):

        url = "http://localhost:8000/api/entity/"

        headers = {
            'api-key': "43d5d4a6b88c4c2ea303188fcbc3385f79d1e961",
            'cache-control': "no-cache",
            'postman-token': "c0ef09c6-ad86-b7f4-52be-fe3c0f6d1238"
        }
        response = None

        if args != None:
            response = requests.get(url, headers=headers, params=args)
        else:
            response = requests.get(url, headers)

        parsedJson = json.loads(response.text)

        return parsedJson

    # ARGS
    # -object                        (where is the shampoo ?)
    # RETURN : room
    def arena_where_located(self, f_arg=None):
        print('entity :', 'arena_where_located')

        object=""
        for arg in f_arg:
            if(arg.get('entity')=='object'):
                object+=arg.get('value')
                object+=" "

        object = object.strip()
        print("object :", object)
        # Try with entityClass first, if nothing, try with entityName
        arg = {"entityCategory": object}
        answer = self.wonderland_get_first_entity(arg)

        if len(answer) == 0:
            arg = {"entityClass": object}
            answer = self.wonderland_get_first_entity(arg)

        container = ""

        if len(answer) > 0:
            while(answer.get('entityCategory') != "Rooms"):
                arg = {"entityId": answer.get('entityContainer')}
                answer = self.wonderland_get_first_entity(arg)
                print("Container : ", answer.get('entityClass'))
                container += " in the " + answer.get('entityClass')

        if container != "":
            toSay = "The " + object + " is" + container
        else:
            toSay = "I'm sorry, I don't know where the " + object + " is"

        return toSay

    # ARGS
    # -object                        (how many chairs ?)
    # -object && object(room)        (how many chairs in the kitchen ?)
    # -object && object(placement)   (how many apple in the fridge ?)
    # RETURN : count
    def arena_count_object(self, f_arg=None):
        print('entity :', 'arena_count_object')
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))


        if len(f_arg) == 1:
            if f_arg[0].get('entity') == "object":
                arg = {"entityClass": f_arg[0].get('value')}
                if len(self.wonderland_get_entity(arg)) != 0:
                    sentence = "There's " + str(len(self.wonderland_get_entity(arg))) + " " + f_arg[0].get('value')
                    return sentence
                else:
                    arg = {"entityCategory": f_arg[0].get('value')}
                    sentence = "There's " + str(len(self.wonderland_get_entity(arg))) + " " + f_arg[0].get('value')
                    return sentence

        elif len(f_arg) == 2:
            if f_arg[0].get('entity') == "object" and f_arg[0].get('entity') == "object":
                arg = {"entityClass": f_arg[0].get('value')}

                # requete wonderland => trouver id du container
                request_arg = {"entityClass": f_arg[1].get('value')}
                answer = self.wonderland_get_first_entity(request_arg)
                container_id = answer.get('entityId')

                count = 0
                id = -1

                # recursive search for container
                objects = self.wonderland_get_entity(arg)
                for object in objects:
                    id = object.get('entityContainer')
                    if id == container_id:
                        count += 1
                    else:
                        while id != container_id and id != None:
                            id = object.get('entityContainer')
                            if id == container_id:
                                count += 1
                            object = self.wonderland_get_first_entity({"entityId": id})

                if count == 0:
                    arg = {"entityCategory": f_arg[0].get('value')}
                    count = 0
                    id = -1

                    # recursive search for container
                    objects = self.wonderland_get_entity(arg)
                    for object in objects:
                        id = object.get('entityContainer')
                        if id == container_id:
                            count += 1
                        else:
                            while id != container_id and id != None:
                                id = object.get('entityContainer')
                                if id == container_id:
                                    count += 1
                                object = self.wonderland_get_first_entity({"entityId": id})

                return "There's " + str(count) + " " + f_arg[0].get('value') + " in the " + f_arg[1].get('value')





    # ARGS
    # -object                        (how many girls ?)
    # -object && gesture             (how many man standing ?)      #TODO
    # -object && color               (how many boys wearing blue ?)
    # RETURN : count
    def crowd_count(self, f_arg=None):
        print('entity: ','crowd_count')
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        requestArg = {"entityClass":"person"}
        answer = self.wonderland_get_entity(requestArg)

        personType = {
            "girl":"girl",
            "girls":"girl",
            "women":"girl",
            "woman":"girl",
            "men":"boy",
            "boys":"boy",
            "boy":"boy",
            "man":"boy",
            "people":""
        }

        if len(f_arg) == 1:
            count = 0
            for person in answer:
                if f_arg[0].get('value') == "people":
                    count += 1
                elif person.get('entityName') == personType.get(f_arg[0].get('value')):
                    count = count + 1
            return "There's " + str(count) + " " + f_arg[0].get('value')

        if len(f_arg) == 2:
            if f_arg[1].get('entity') == "color":
                count = 0
                for person in answer:
                    if f_arg[0].get('value') == "people" and person.get('entityColor') == f_arg[1].get('value'):
                        count += 1
                    if person.get('entityName') == personType.get(f_arg[0].get('value')) \
                            and person.get('entityColor') == f_arg[1].get('value'):
                        count = count + 1
                return "There's " + str(count) + " " + f_arg[0].get('value') + " wearing " + f_arg[1].get('value')

            if f_arg[1].get('entity') == "gesture":
                count = 0
                return "I can't answer this question yet"

    # ARGS
    # -gesture && person_type        (tell me if the standing person was a male) #TODO
    # -gesture && person_type (x2)   (is the person pointing a girl or a boy ?) #TODO
    # RETURN : yes/no || person_type
    def crowd_person_gesture(self, f_arg=None):
        print('entity: ', 'crowd_person_gesture')
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        # TODO : requete wonderland => get person with param(gesture)


    # ARGS
    # -object                        (What's the colour of the chocolate egg?)
    # RETURN : color
    def arena_color_object(self, f_arg=None):
        print('entity: ', 'arena_color_object')
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        if len(f_arg) > 0:
            if f_arg[0].get('entity') == "object":
                arg = {"entityClass": f_arg[0].get('value')}
                object = self.wonderland_get_first_entity(arg)
                return "The " + f_arg[0].get('value') + " is " + object.get('entityColor')


    # ARGS
    # -object                        (What objects are stored in the living table?)
    # RETURN : object(s)
    def arena_which_object(self, f_arg=None):
        print('entity: ', 'arena_which_object')
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        if len(f_arg) > 0:
            #print(f_arg[0])

            # requete wonderland => trouver id du container
            request_arg = {"entityClass": f_arg[0].get('value')}
            answer = self.wonderland_get_first_entity(request_arg)
            #print(answer.get('entityId'))

            objects = ""

            answer2 = self.wonderland_get_entity()
            for object in answer2:
                if object.get("entityContainer") == answer.get('entityId'):
                    objects += ", " + object.get("entityClass")

            if objects == "":
                return ("I can't find any objects in the " + f_arg[0].get('value'))
            else:
                return ("In the " + f_arg[0].get('value') + ", theres :" + objects)

        return "I can't find any objects"

    # ARGS
    # -object                        (To which category belong the melon?)
    # -object && object              (Do the bowl and chocolate bar belong to the same category?)
    # RETURN : category || yes/no
    def object_category(self, f_arg=None):
        print('entity: ', 'object_category')
        for arg in f_arg:
            print(arg.get('entity'), ":", arg.get('value'))

        if len(f_arg) == 1:
            if f_arg[0].get('entity') == "object":
                answer = self.wonderland_get_first_entity({"entityClass": f_arg[0].get('value')})
                return "The " + f_arg[0].get('value') + " belong to the " + answer.get('entityCategory')
        if len(f_arg) == 2:
            if f_arg[0].get('entity') == "object" and f_arg[1].get('entity') == "object":
                answer1 = self.wonderland_get_first_entity({"entityClass": f_arg[0].get('value')})
                answer2 = self.wonderland_get_first_entity({"entityClass": f_arg[1].get('value')})
                if len(answer1) == 0 or len(answer2) == 0:
                    return "I'm sorry, I don't know these objects"
                if answer1.get('entityCategory') == answer2.get('entityCategory'):
                    return "Yes, the " + f_arg[0].get('value') + " and the " + f_arg[1].get('value') + " belongs to the same category"
                else:
                    return "No, the " + f_arg[0].get('value') + " and the " + f_arg[1].get(
                        'value') + " does not belongs to the same category"


    # ARGS
    # -adjective                     (Which is the biggest object?)
    # -object && object && adjective (Between the mints and apple, which one is bigger?)
    # -adjective && object(category) (Which is the lightest snacks?)
    def object_adjective(self, f_arg=None):
        print('entity: ', 'object_adjective')

        object1 = ""
        object2 = ""
        adjective = ""

        for args in f_arg:
            print(args.get('entity'), ":", args.get('value'))
            if args.get('entity') == 'object':
                if object1 == "":
                    object1 = args.get('value')
                else:
                    object2 = args.get('value')
            elif args.get('entity') == 'adjective':
                adjective = args.get('value')

        answer1 = self.wonderland_get_first_entity({"entityClass": object1})
        answer2 = self.wonderland_get_first_entity({"entityClass": object2})

        if adjective == "heaviest" or adjective == "heavier":

            if len(f_arg) == 1 or object1 == "object":
                answer = self.wonderland_get_entity()
                heaviestObject = ""
                weight = 0

                for object in answer:
                    if int(object.get('entityWeight') or 0) > weight and object.get('entityName') == 'object':
                        heaviestObject = object.get('entityClass')
                        weight = object.get('entityWeight')

                return "The heaviest " + object1 + " is the " + heaviestObject

            if len(f_arg) == 2:
                arg = {'entityCategory' : object1}
                answer = self.wonderland_get_entity(arg)
                heaviestObject = ""
                weight = 0

                for object in answer:
                    if object.get('entityWeight') > weight:
                        heaviestObject = object.get('entityClass')
                        weight = object.get('entityWeight')

                return "The heaviest " + object1 + " is the " + heaviestObject

            if len(f_arg) == 3:
                if answer1.get('entityWeight') > answer2.get('entityWeight'):
                    return "The " + object1 + " is heavier than the " + object2
                else:
                    return "The " + object2 + " is heavier than the " + object1

        if adjective == "smallest" or adjective == "smaller":

            if len(f_arg) == 1 or object1 == "object":
                answer = self.wonderland_get_entity()
                smallestObject = ""
                size = 99999

                for object in answer:
                    if int(object.get('entitySize') or 99999) < size and object.get('entityName') == 'object':
                        smallestObject = object.get('entityClass')
                        size = object.get('entitySize')

                return "The smallest " + object1 + " is the " + smallestObject

            if len(f_arg) == 2:

                arg = {'entityCategory' : object1}
                answer = self.wonderland_get_entity(arg)
                smallestObject = ""
                size = 9999999

                for object in answer:
                    if object.get('entitySize') < size:
                        smallestObject = object.get('entityClass')
                        size = object.get('entitySize')

                return "The smallest " + object1 + " is the " + smallestObject

            if len(f_arg) == 3:
                if answer1.get('entitySize') < answer2.get('entitySize'):
                    return "The " + object1 + " is smaller than the " + object2
                else:
                    return "The " + object2 + " is smaller than the " + object1


        if adjective == "biggest" or adjective == "bigger":

            if len(f_arg) == 1 or object1 == "object":
                answer = self.wonderland_get_entity()
                biggestObject = ""
                size = 0

                for object in answer:
                    if int(object.get('entitySize') or 0) > size and object.get('entityName') == 'object':
                        biggestObject = object.get('entityClass')
                        size = object.get('entitySize')

                return "The biggest " + object1 + " is the " + biggestObject

            if len(f_arg) == 2:
                arg = {'entityCategory' : object1}
                answer = self.wonderland_get_entity(arg)
                biggestObject = ""
                size = 0

                for object in answer:
                    if object.get('entitySize') > size:
                        biggestObject = object.get('entityClass')
                        size = object.get('entitySize')

                return "The biggest " + object1 + " is the " + biggestObject

            if len(f_arg) == 3:
                if answer1.get('entitySize') > answer2.get('entitySize'):
                    return "The " + object1 + " is bigger than the " + object2
                else:
                    return "The " + object2 + " is bigger than the " + object1

        if adjective == "lightest" or adjective == "lighter":

            if len(f_arg) == 1 or object1 == "object":
                answer = self.wonderland_get_entity()
                lightestObject = ""
                weight = 99999

                for object in answer:
                    if int(object.get('entityWeight') or 99999) < weight and object.get('entityName') == 'object':
                        lightestObject = object.get('entityClass')
                        weight = object.get('entityWeight')

                return "The lightest " + object1 + " is the " + lightestObject

            if len(f_arg) == 2:
                arg = {'entityCategory': object1}
                answer = self.wonderland_get_entity(arg)
                lightestObject = ""
                weight = 999999

                for object in answer:
                    if object.get('entityWeight') < weight:
                        lightestObject = object.get('entityClass')
                        weight = object.get('entityWeight')

                return "The lightest " + object1 + " is the " + lightestObject

            if len(f_arg) == 3:
                if answer1.get('entityWeight') > answer2.get('entityWeight'):
                    return "The " + object1 + " is heavier than the " + object2
                else:
                    return "The " + object2 + " is heavier than the " + object1
            if len(f_arg) == 3:
                if answer1.get('entityWeight') < answer2.get('entityWeight'):
                    return "The " + object1 + " is lighter than the " + object2
                else:
                    return "The " + object2 + " is lighter than the " + object1




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
        print(response)
        entities = response.get('entities')

        sentence = self.rasa.intent_functions[response.get('intent').get('name')](entities)

        # printing question/answer
        #print('*' * 40)
        #print('Q :', question)
        return(sentence)

    def handle_answer_question(self, req):
        print("start wm_nlu")
        question = str(req.str.data)
        q = PredefinedQuestion()
        rospack = rospkg.RosPack()
        q.loadQuestions(rospack.get_path('wm_nlu')+"/script/Questions.xml")
        q.callback(question)
        answer = ""
        # QUESTIONS PREDEFINIES
        if q.WEIGHT[max(q.WEIGHT, key=q.WEIGHT.get)] / len(question.split()) * 100 > 75:
            print('*' * 40)
            print('Q :', question)
            answer =  q.QUESTIONS[max(q.WEIGHT, key=q.WEIGHT.get)]
            print('A :', q.QUESTIONS[max(q.WEIGHT, key=q.WEIGHT.get)])

        # QUESTIONS CROWDS, PEOPLE, OBJECTS
        else:
            answer = self.call_rasa(question.decode('utf-8'))
            print('*' * 40)
            print('Q :', question)
            print('A :', answer)

        return AnswerQuestionResponse(String(answer))




    def answer_question_server(self):
        rospy.init_node('answer_question_server')
        s = rospy.Service('answer_question', AnswerQuestion, self.handle_answer_question)
        print "Ready to answer questions."
        rospy.spin()


if __name__ == "__main__":
    q = Question()
    q.answer_question_server()


    ###############################
    # QUESTIONS EXAMPLES
    ###############################
    #
    # What objects are stored in the fridge ?
    # Do the banana and apple belong to the same category?
    # Do the coke and apple belong to the same category?
    # To which category belong the melon?
    # Where is the shampoo ?
    # Where is the water ?
    # How many drinks ?
    # How many apple ?
    # How many person ?
    # How many drinks in the kitchen ?
