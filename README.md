# wm_nlu
ROS wrapper for rasa nlu with datasets for GPSR and SPR robocup@home challenge.

# Service
* /gpsr_receive_action
* /answer_question

## Installation

### General
```bash
pip install sklearn_crfsuite sklearn --user
pip install testresources rasa_nlu spacy --user
pip install -U scikit-learn scipy sklearn-crfsuite --user
python -m spacy download en_core_web_md
python -m spacy link en_core_web_md en
```
### For the SPR challenge
* Wonderland is needed to run the SPR challenge
* Follow https://github.com/walkingmachine/wonderland#installation for the installation

## Prepare the data
* 2 datasets are used, robocup_gpsr.json and robocup_spr.json
* If you want to modify the current dataset, you can use the online tool https://rasahq.github.io/rasa-nlu-trainer/

## How to test wm_nlu
### For the SPR challenge
```bash
rosrun wm_nlu spr_nlu.py
```
### For the GPSR challenge
```bash
rosrun wm_nlu gpsr_nlu.py
```
