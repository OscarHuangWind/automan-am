#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  2 10:26:23 2021

@author: oscar
"""
import numpy as np
np.random.BitGenerator = np.random.bit_generator.BitGenerator
import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as ctrl
# import pandas as pd

class Authority():
    
    def __init__(self):
        # New Antecedent/Consequent objects hold universe variables and membership
        self.arf = ctrl.Antecedent(np.arange(1.0, 5.0, 0.01), 'arf')
        self.steering = ctrl.Antecedent(np.arange(0, 0.45, 0.01), 'steering')
        self.control_authority = ctrl.Consequent(np.arange(0.0, 1.0, 0.01), 'control_authority')
        self.rule_list = []
        self.control_authority_ctrl = ctrl.ControlSystem()
        self.authority = ctrl.ControlSystemSimulation(self.control_authority_ctrl)
        self.Initialization()
        
    def Initialization(self):
        # Generate fuzzy membership functions
        self.arf['low'] = fuzz.trapmf(self.arf.universe, [1.0, 1.0, 1.5, 2.5])
        self.arf['medium'] = fuzz.trimf(self.arf.universe, [1.5, 2.5, 3.5])
        self.arf['high'] = fuzz.trapmf(self.arf.universe, [2.5, 3.0, 5.0, 5.0])
        
        self.steering['very small'] = fuzz.trapmf(self.steering.universe, [0.0, 0.0, 0.15, 0.2])
        self.steering['small'] = fuzz.trimf(self.steering.universe, [0.15, 0.2, 0.25])
        self.steering['medium'] = fuzz.trimf(self.steering.universe, [0.2, 0.25, 0.3])
        self.steering['big'] = fuzz.trimf(self.steering.universe, [0.25, 0.3, 0.35])
        self.steering['very big'] = fuzz.trapmf(self.steering.universe, [0.3, 0.35, 0.45, 0.45])

        self.control_authority['zero'] = fuzz.trapmf(self.control_authority.universe, [0.0, 0.0, 0.01, 0.01])
        self.control_authority['very low'] = fuzz.trimf(self.control_authority.universe, [0.0, 0.1, 0.2])
        self.control_authority['low'] = fuzz.trimf(self.control_authority.universe, [0.1, 0.25, 0.4])
        self.control_authority['medium'] = fuzz.trimf(self.control_authority.universe, [0.2, 0.4, 0.6])
        self.control_authority['high'] = fuzz.trimf(self.control_authority.universe, [0.4, 0.6, 0.8])
        self.control_authority['very high'] = fuzz.trapmf(self.control_authority.universe, [0.7, 0.8, 1.0, 1.0])
        
        rule1 = ctrl.Rule(self.arf['low'] & self.steering['very small'], self.control_authority['very high'])
        rule2 = ctrl.Rule(self.arf['low'] & self.steering['small'], self.control_authority['very high'])
        rule3 = ctrl.Rule(self.arf['low'] & self.steering['medium'], self.control_authority['high'])
        rule4 = ctrl.Rule(self.arf['low'] & self.steering['big'], self.control_authority['high'])
        rule5 = ctrl.Rule(self.arf['low'] & self.steering['very big'], self.control_authority['medium'])

        rule6 = ctrl.Rule(self.arf['medium'] & self.steering['very small'], self.control_authority['high'])
        rule7 = ctrl.Rule(self.arf['medium'] & self.steering['small'], self.control_authority['high'])
        rule8 = ctrl.Rule(self.arf['medium'] & self.steering['medium'], self.control_authority['medium'])
        rule9 = ctrl.Rule(self.arf['medium'] & self.steering['big'], self.control_authority['low'])
        rule10 = ctrl.Rule(self.arf['medium'] & self.steering['very big'], self.control_authority['very low'])
        
        rule11 = ctrl.Rule(self.arf['high'] & self.steering['very small'], self.control_authority['medium'])
        rule12 = ctrl.Rule(self.arf['high'] & self.steering['small'], self.control_authority['low'])
        rule13 = ctrl.Rule(self.arf['high'] & self.steering['medium'], self.control_authority['very low'])
        rule14 = ctrl.Rule(self.arf['high'] & self.steering['big'], self.control_authority['zero'])
        rule15 = ctrl.Rule(self.arf['high'] & self.steering['very big'], self.control_authority['zero'])
        
        self.rule_list = [rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15]
        self.control_authority_ctrl = ctrl.ControlSystem(self.rule_list)
        self.authority = ctrl.ControlSystemSimulation(self.control_authority_ctrl)

    def SetInput(self, input1, input2):
        self.authority.input['arf'] = input1
        self.authority.input['steering'] = input2

    def ComputeAuthority(self):
        self.authority.compute()
        
    def GetAuthority(self):
        return self.authority.output['control_authority']

# author = Authority()
# author.steering.view()
# author.arf.view()
# author.control_authority.view()
# plt.show()

# authority_matrix = np.zeros((9, 10))

# for i in range(0, 9):
#     for j in range(0, 10):
#         author.authority.input['arf'] = 1.0 + i * 0.5
#         author.authority.input['steering'] = 0.0 + j * 0.05
#         author.authority.compute()
#         author.control_authoritys_final = author.authority.output['control_authority']
#         authority_matrix[i][j] = author.control_authoritys_final

# authority_matrix[authority_matrix < 0.05] = 0

# df = pd.DataFrame({"Deviation:0 rad/s":[x for x in authority_matrix[:,0]],
#                    "Deviation:0.05 rad/s":[x for x in authority_matrix[:,1]],
#                    "Deviation:0.1 rad/s":[x for x in authority_matrix[:,2]],
#                    "Deviation:0.15 rad/s":[x for x in authority_matrix[:,3]],
#                    "Deviation:0.2 rad/s":[x for x in authority_matrix[:,4]],
#                    "Deviation:0.25 rad/s":[x for x in authority_matrix[:,5]],
#                    "Deviation:0.3 rad/s":[x for x in authority_matrix[:,6]],
#                    "Deviation:0.35 rad/s":[x for x in authority_matrix[:,7]],
#                    "Deviation:0.4 rad/s":[x for x in authority_matrix[:,8]],
#                    "Deviation:0.45 rad/s":[x for x in authority_matrix[:,9]],})

# df.index = ['APF=1', 'APF=1.5', 'APF=2', 'APF=2.5', 'APF=3', 'APF=3.5', 'APF=4', 'APF=4.5', 'APF=5']
# Crunch the numbers
# author.authority.compute()
# author.control_authoritys_final = author.authority.output['control_authority']
# print(author.control_authoritys_final)
