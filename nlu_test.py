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


def wonderland_get_entity( args = None ):

    url = "http://localhost:8000/api/entity/"

    headers = {
        'api-key': "43d5d4a6b88c4c2ea303188fcbc3385f79d1e961",
        'cache-control': "no-cache",
        'postman-token': "c0ef09c6-ad86-b7f4-52be-fe3c0f6d1238"
    }
    response = None
    if args != None:
        response = requests.get(url, headers, args)
    else:
        response = requests.get(url, headers)

    parsedJson = json.loads(response.text)
    if len(parsedJson) > 0:
        print(parsedJson[0])


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
    print(object)

    ## this is only a test response
    print("REPSONSE :", "you can find the", object, "in the kitchen")
    # TODO : requete wonderland => get room from object
    wonderland_get_entity()

# ARGS
# -object                        (how many chairs ?)
# -object && room                (how many chairs in the kitchen ?)
# -object && object(placement)   (how many apple in the fridge ?)
# RETURN : count
def arena_count_object(f_arg=None):
    print('entity : ', 'arena_count_object')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))

    # TODO : requete wonderland => get list of object with param(room || container)


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

# ARGS
# -object                        (To which category belong the melon?)
# -object && object              (Do the bowl and chocolate bar belong to the same category?)
# RETURN : category || yes/no
def object_category(f_arg=None):
    print('entity : ', 'object_category')
    for arg in f_arg:
        print(arg.get('entity'), ":", arg.get('value'))

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
    response = interpreter.parse(question)
    entities = response.get('entities')
    intent_functions[response.get('intent').get('name')](entities)

if __name__ == "__main__":
    print("start test_rasa")
    test_rasa("Where can I find a drinks?")