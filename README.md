# wm_nlu
ROS wrapper for rasa nlu. Mainly used for SPR robocup@home challenge.  
*Currently not a ROS package.*

## Installation
* pip install rasa_nlu
* pip install rasa_nlu[spacy]
* python -m spacy download en_core_web_md
* python -m spacy link en_core_web_md en

## Status

Currently, wm_nlu will query rasa_nlu to get the sentences entities. It will then query [wonderland](https://github.com/walkingmachine/wonderland) and return the answer to the answer.

## Future changes

Once the first version will be working, the wonderland part will be seperated from wm_nlu. Another package will be in charged of converting the ouput from wm_nlu to query wonderland database.
