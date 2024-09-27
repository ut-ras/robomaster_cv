# Readme for Object Log

## Problems

### Associating bounding boxes to armor plates

Approaches

1. An arbitrary error in the prediction and then associating the next one within these bounds
2. ML approach (potential KNN or SVM based on historical prediction data)
3. Look for other papers / approaches for doing this

## Going with Particle Filter Approach / keeping what we had last year

# Stories

Lets first write the prototypes of each of these and leave header comments for each function so other people know how to structure them.

## Migrating Object Log from Python to C++

- Armor Plate class:
  * Tanay
- Object Log class
  * David
- Kalman Filter Class
  * Tanay

## Other tasks:
- Testing code
