#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  6 14:09:21 2021

@author: oscar
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  2 10:26:23 2021

@author: oscar
"""
import numpy as np
import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class Authority():
    
    def __init__(self):
        # New Antecedent/Consequent objects hold universe variables and membership
        self.arf = ctrl.Antecedent(np.arange(1.0, 5.0, 0.01), 'arf')
        self.attention = ctrl.Antecedent(np.arange(0, 1.0, 0.01), 'attention')
        self.control_authority = ctrl.Consequent(np.arange(0.0, 1.0, 0.01), 'control_authority')
        self.rule_list = []
        self.control_authority_ctrl = ctrl.ControlSystem()
        self.authority = ctrl.ControlSystemSimulation(self.control_authority_ctrl)
        self.Initialization()
        
    def Initialization(self):
        # Generate fuzzy membership functions
        self.arf['small'] = fuzz.trapmf(self.arf.universe, [1.0, 1.0, 1.5, 2.5])
        self.arf['medium'] = fuzz.trimf(self.arf.universe, [1.5, 2.5, 3.5])
        self.arf['large'] = fuzz.trapmf(self.arf.universe, [2.5, 2.5, 5.0, 5.0])
        
        self.attention['concentrative'] = fuzz.trapmf(self.attention.universe, [0.0, 0.0, 0.2, 0.5])
        self.attention['partially divergent'] = fuzz.trapmf(self.attention.universe, [0.2, 0.5, 0.8, 0.8])
        self.attention['divergent'] = fuzz.trapmf(self.attention.universe, [0.8, 0.8, 1.0, 1.0])
        
        self.control_authority['zero'] = fuzz.trapmf(self.control_authority.universe, [0.0, 0.0, 0.0, 0.0])
        self.control_authority['very low'] = fuzz.trimf(self.control_authority.universe, [0.0, 0.1, 0.2])
        self.control_authority['low'] = fuzz.trimf(self.control_authority.universe, [0.1, 0.25, 0.4])
        self.control_authority['medium'] = fuzz.trimf(self.control_authority.universe, [0.2, 0.4, 0.6])
        self.control_authority['high'] = fuzz.trimf(self.control_authority.universe, [0.4, 0.6, 0.8])
        self.control_authority['very high'] = fuzz.trapmf(self.control_authority.universe, [0.7, 0.8, 1.0, 1.0])
        
        rule1 = ctrl.Rule(self.arf['small'] & self.attention['concentrative'], self.control_authority['very high'])
        rule2 = ctrl.Rule(self.arf['small'] & self.attention['partially divergent'], self.control_authority['medium'])
        rule3 = ctrl.Rule(self.arf['small'] & self.attention['divergent'], self.control_authority['zero'])
        
        rule4 = ctrl.Rule(self.arf['medium'] & self.attention['concentrative'], self.control_authority['high'])
        rule5 = ctrl.Rule(self.arf['medium'] & self.attention['partially divergent'], self.control_authority['medium'])
        rule6 = ctrl.Rule(self.arf['medium'] & self.attention['divergent'], self.control_authority['zero'])
        
        rule7 = ctrl.Rule(self.arf['large'] & self.attention['concentrative'], self.control_authority['low'])
        rule8 = ctrl.Rule(self.arf['large'] & self.attention['partially divergent'], self.control_authority['very low'])
        rule9 = ctrl.Rule(self.arf['large'] & self.attention['divergent'], self.control_authority['zero'])
        
        self.rule_list = [rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9]
        self.control_authority_ctrl = ctrl.ControlSystem(self.rule_list)
        self.authority = ctrl.ControlSystemSimulation(self.control_authority_ctrl)

    def SetInput(self, input1, input2):
        self.authority.input['arf'] = input1
        self.authority.input['attention'] = input2

    def ComputeAuthority(self):
        self.authority.compute()
        
    def GetAuthority(self):
        return self.authority.output['control_authority']

# authority = np.zeros((9, 11))

# # self.control_authority.input['arf'] = np.arange(1.0, 5.5, 0.5)#6.5
# # self.control_authority.input['attention'] = np.arange(0.0, 1.1, 0.1)#9.8

# for i in range(0, 9):
#     for j in range(0, 11):
#         self.control_authority.input['arf'] = 1.0 + i * 0.5
#         self.control_authority.input['attention'] = 0.0 + j * 0.1
#         self.control_authority.compute()
#         self.control_authoritys_final = self.control_authority.output['control_authority']
#         authority[i][j] = self.control_authoritys_final

# Crunch the numbers
# self.control_authority.compute()
# self.control_authoritys_final = self.control_authority.output['self.control_authority']
# print(self.control_authoritys_final)
