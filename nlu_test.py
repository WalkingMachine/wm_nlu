#!/usr/bin/env python

import requests
import sys
import os

from rasa_nlu.training_data import load_data
from rasa_nlu.config import RasaNLUModelConfig
from rasa_nlu.model import Trainer
from rasa_nlu import config
from rasa_nlu.model import Metadata, Interpreter
import json

def wonderland_get_first_entity( args = None ):

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

def wonderland_get_entity( args = None ):

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
def arena_where_located(f_arg=None):
    print('entity : ', 'arena_where_located')

    object=""
    for arg in f_arg:
        if(arg.get('entity')=='object'):
            object+=arg.get('value')
            object+=" "

    object = object.strip()
    print("object :", object)
    # Try with entityClass first, if nothing, try with entityName
    arg = {"entityCategory": object}
    answer = wonderland_get_first_entity(arg)

    if len(answer) == 0:
        arg = {"entityClass": object}
        answer = wonderland_get_first_entity(arg)

    container = ""

    if len(answer) > 0:
        while(answer.get('entityCategory') != "Rooms"):
            arg = {"entityId": answer.get('entityContainer')}
            answer = wonderland_get_first_entity(arg)
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
def arena_count_object(f_arg=None):
    print('entity : ', 'arena_count_object')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))

    # requete wonderland => get list of object with param(room || container)
    if len(f_arg) == 1:
        if f_arg[0].get('entity') == "object":
            arg = {"entityClass": f_arg[0].get('value')}
            sentence = "There's " + str(len(wonderland_get_entity(arg))) + " " + f_arg[0].get('value')
            return sentence

    elif len(f_arg) == 2:
        if f_arg[0].get('entity') == "object" and f_arg[0].get('entity') == "object":
            arg = {"entityClass": f_arg[0].get('value')}

            # requete wonderland => trouver id du container
            request_arg = {"entityClass": f_arg[1].get('value')}
            answer = wonderland_get_first_entity(request_arg)
            print(answer.get('entityId'))





# ARGS
# -object                        (how many girls ?)
# -object && gesture             (how many man standing ?)
# -object && color               (how many boys wearing blue ?)
# RETURN : count
def crowd_count(f_arg=None):
    print('entity : ','crowd_count')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))

    # TODO : requete wonderland => get list of persons with param(gesture || color)

# ARGS
# -gesture && person_type        (tell me if the standing person was a male)
# -gesture && person_type (x2)   (is the person pointing a girl or a boy ?)
# RETURN : yes/no || person_type
def crowd_person_gesture(f_arg=None):
    print('entity : ', 'crowd_person_gesture')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))

    # TODO : requete wonderland => get person with param(gesture)


# ARGS
# -object                        (What's the colour of the chocolate egg?)
# RETURN : color
def arena_color_object(f_arg=None):
    print('entity : ', 'arena_color_object')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))

    # TODO : requete wonderland => get list of persons with param(gesture || color)


# ARGS
# -object                        (What objects are stored in the living table?)
# RETURN : object(s)
def arena_which_object(f_arg=None):
    print('entity : ', 'arena_which_object')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))

    if len(f_arg) > 0:
        #print(f_arg[0])

        # requete wonderland => trouver id du container
        request_arg = {"entityClass": f_arg[0].get('value')}
        answer = wonderland_get_first_entity(request_arg)
        #print(answer.get('entityId'))

        objects = ""

        answer2 = wonderland_get_entity()
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
def object_category(f_arg=None):
    print('entity : ', 'object_category')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))

    if len(f_arg) == 1:
        if f_arg[0].get('entity') == "object":
            answer = wonderland_get_first_entity({"entityClass": f_arg[0].get('value')})
            return "The " + f_arg[0].get('value') + " belong to the " + answer.get('entityCategory')
    if len(f_arg) == 2:
        if f_arg[0].get('entity') == "object" and f_arg[1].get('entity') == "object":
            answer1 = wonderland_get_first_entity({"entityClass": f_arg[0].get('value')})
            answer2 = wonderland_get_first_entity({"entityClass": f_arg[1].get('value')})
            if len(answer1) == 0 or len(answer2) == 0:
                return "I'm sorry, I don't know these objects"
            if answer1.get('entityCategory') == answer2.get('entityCategory'):
                return "Yes, the " + f_arg[0].get('value') + " and the " + f_arg[1].get('value') + " belongs to the same category"
            else:
                return "No, the " + f_arg[0].get('value') + " and the " + f_arg[1].get(
                    'value') + " does not belongs to the same category"


# ARGS
# -object && object && adjective (Between the mints and apple, which one is bigger?)
# -adjective && object(category) (Which is the lightest snacks?)
def object_adjective(f_arg=None):
    print('entity : ', 'object_adjective')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))


# Dict containing the entity and their function
intent_functions = {
    'arena_where_located': arena_where_located,
    'arena_count_object': arena_count_object,
    'crowd_count': crowd_count,
    'crowd_person_gesture': crowd_person_gesture,
    'arena_color_object': arena_color_object,
    'arena_which_object': arena_which_object,
    'object_category': object_category,
    'object_adjective': object_adjective
}

def test_rasa(question):

    sys.stderr = open(os.devnull, 'w')

    # Load the config files
    print("Loading config files...")
    training_data = load_data('robocup_spr.json')
    trainer = Trainer(config.load("config_spacy.yml"))

    print("Training the model...")
    # Train the model based on the robocup_spr.json file
    trainer.train(training_data)
    # Returns the directory the model is stored in
    model_directory = trainer.persist('./projects/default/')

    print("Loading the model...")
    interpreter = Interpreter.load(model_directory)

    # return the JSON as a dict
    print("Calling rasa_nlu...")
    print('*' * 40)
    response = interpreter.parse(question)
    entities = response.get('entities')
    sentence = intent_functions[response.get('intent').get('name')](entities)

    # printing question/answer
    print('*' * 40)
    print('Q :', question)
    print('A :', sentence)

if __name__ == "__main__":
    print("start wm_nlu")
    test_rasa("where is the water ?")

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
    #