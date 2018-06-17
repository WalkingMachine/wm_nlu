# wm_nlu
ROS wrapper for rasa nlu. Mainly used for SPR robocup@home challenge.  
*Currently not a ROS package.*

## Installation

```bash
python -m pip install sklearn_crfsuite sklearn --user
python -m pip install testresources rasa_nlu spacy --user
python -m spacy download en_core_web_md
python -m spacy link en_core_web_md en
```

## Prepare the data
* https://rasahq.github.io/rasa-nlu-trainer/

## How to test wm_nlu
* Follow the installation process
* `git clone http://github.com/walkingmachine/wonderland` in your ROS workspace
* in wonderland repo : `git checkout robocup2018`
* catkin_make your workspace
* start wonderland : `python manage.py runserver`
* `rosrun wm_nlu spr_nlu`
or  
* `rosrun wm_nlu gpsr_nlu`

## Status

Currently, wm_nlu will query rasa_nlu to get the sentences entities. It will then query [wonderland](https://github.com/walkingmachine/wonderland) and return the answer to the answer.

## Future changes

Once the first version will be working, the wonderland part will be seperated from wm_nlu. Another package will be in charged of converting the ouput from wm_nlu to query wonderland database.
