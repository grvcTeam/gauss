#!/usr/bin/env python
# Emergency Management module with resolution of unexpected events happened during the flight. It is considered a
# U-space framework
# Main author: Carlos Capitan Fernandez.

# UAS CLASS DEFINITION


class UAS:
    def __init__(self, ID, frame, range, priority, FP, pose, FG, CONOP, conformance):
        self.ID = ID
        self.frame = frame  # The UAS is Fixed or rotary wing (RW or FW)
        self.range = range  # Distance which the UAS can keep flying (in meters)
        self.priority = priority  # Priority of the mission (HIGH or LOW)
        self.FP = FP  # Flight Plan (List of way points)
        self.pose = pose  # Current pose
        self.FG = FG  # Flight Geometry
        self.CONOP = CONOP  # Operation definition
        self.conformance = conformance  # Needed?. It would define if the UAS is in normal, undesired or un-recovered
        # state


# UAS OBJECTS CREATION

UAS_1 = UAS(1, 'RW', 10, 'LOW', [[0, 0, 1], [0, 1, 1], [0, 0, 0]], [0, 0, 0], [1, 1, 1], 'Traffic monitoring',
            'normal')

UAS_2 = UAS(2, 'RW', 5, 'HIGH', [[2, 2, 1], [0, 2, 1], [0, 2, 0]], [2, 2, 0], [1, 1, 1], 'Package delivery',
            'normal')  # se sale del FG

uas_list = [UAS_1, UAS_2]


class Threat:
    def __init__(self, ID, name, severity):
        self.ID = ID
        self.name = name
        self.severity = severity


threat_1 = Threat(1, 'UAS_IN_CV', 1)
threat_2 = Threat(2, 'UAS_OUT_OV', 2)
threat_list = [threat_1, threat_2]

# DICTIONARY WHICH DEFINES WHAT UAS HAS ACTIVE WHICH THREATS


uas_threats = {'uas1': {'UAS_IN_CV': 1},
               'uas2': {'UAS_OUT_OV': 2}
               }


class Actions:
    def __init__(self, FP, FG, t, landing_spots, range, NFP):
        self.FP = FP
        self.FG = FG
        self.t = t
        self.landing_spots = landing_spots
        self.range = range
        self.NFP = NFP

    def callBack2FGAction(self):
        print("Executing Back2FG action")

    def callFollow_pathAction(self):
        print("Executing Follow path action")

    def callLandAction(self):
        print("Executing Land action")

    def callActivate_FTSAction(self):
        print("Executing Activate_FTS action")

    def call_action(self, action):
        if action == "back2FG":
            self.callBack2FGAction()
        elif action == "follow_path":
            self.callFollow_pathAction()
        elif action == "land":
            self.callLandAction()
        elif action == "activate_FTS":
            self.callActivate_FTSAction()


# Hacer una función en la que se calculen los costes de aplicar una acción para cada UAS
cost_actions = {'uas1': {'back2FG': 25,
                         'follow_path': 100,
                         'land': 75,
                         'activate_FTS': 50},

                'uas2': {'back2FG': 50,
                         'follow_path': 25,
                         'land': 75,
                         'activate_FTS': 100}

                }


# Hacer una función que averigue cuál es la acción que tiene un menor coste a la hora de aplicarla

def fittest_action(self):
    min_cost = float('inf')
    for actions in cost_actions[uas]:
        if cost_actions[uas][actions] < min_cost:
            min_cost = cost_actions[uas][actions]
            action = actions
    print(action)


uas1_action = Actions(UAS_1.FP, UAS_1.FG, 5, [0, 0, 0], UAS_1.range, [[1, 1, 1], [2, 2, 2]])
uas2_action = Actions(UAS_2.FP, UAS_2.FG, 10, [0, 0, 0], UAS_2.range, [[2, 2, 2], [2, 2, 2]])

# Calcular la acción más acorde para cada UAS y ejecutarla.
for uas in cost_actions:



def main():
    for uas in cost_actions:
        print(uas)
        fittest_action(uas)
        uas1_action.call_action(fittest_action(uas))
        uas2_action.call_action(fittest_action())


# if __name__ == "__main__":
#    main()
