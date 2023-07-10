
# IMPLEMENTATION LIST

Bellow is proposed a list of implementation that should be performed in order.

## 1. New food entry

Create a new food candidate into the "Fulano" diet list (aka food dataset).

## 2. Creating a new pipeline

"Fulano" is the patient used for this example. Now, create the "Cicrano" pipeline configuration different from Fulano and run both pipelines just by changing the goal parameter.

## 3. Fat scorer heuristic implementation
Implement a fat scorer heuristic using the polymorphism presented in current package.
This heuristic should score food according to its fat level, priority to less fat foods!
The heuristic yaml description should be defined as follows:

```
    0_fat_scorer:
        weight: 1
        threshold: 10 #in grams
```

## 4. Caloric heuristic implementation

Implement a caloric scorer heuristic using the polymorphism presented in current package. 
This heuristic should score food according to its caloric level, priority to less caloric food!


To perform this heuristic, consider the caloric function bellow:

```
cal = [protein_gram*protein_cal_per_gram] + [fat_gram*fat_cal_per_gram] + [fiber_gram*fiber_cal_per_gram]
```

The yaml description should be defined by:

```
    0_caloric_scorer:
        weight: 1
        threshold: 10 #in kcal
        protein_cal_per_gram: 4 #kcal
        fat_cal_per_gram: 9 #kcal
        fiber_cal_per_gram: 2 #kcal
```

## 5. Preload support

The pipeline is implemented only with support to "executeDirect" process, i.e. every time a goal is received the pipeline server load into the memory all 
data from parameter server. This is useful when we need to change the "patient" according to goal request. However, this is redundant when
the same patient pipeline is requested. 

An efficient solution is to uncouple the DirectProcess into PreLoad and Standalone execution. The PreLoad only load into memory the data 
from parameter server and the Standalone execution just run the pipeline. Be notice that, when run the Standalone mode, 
the pipeline need to verify if the patient name is the same which the one used in PreLoad support. 

Considering this, and withou removing the current "executeDirect" process, create the uncoupled solution.

**HELP:** Starts the implementation by setting operation_mode into ```DietEstimationSkill::executeProcess``` method.

## 6. Documentation

For all developed solution, build the on code documentation (trough format comments) and also a markdown readme (called README_COMPLEMENT.md) description the new pipeline capabilities.