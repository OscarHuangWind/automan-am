#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep  6 14:29:27 2021

@author: oscar
"""

# arf = ctrl.Antecedent(np.arange(1.0, 5.0, 0.01), 'arf')
# steer_dev = ctrl.Antecedent(np.arange(0, 0.5, 0.01), 'steer_dev')
# control_authority = ctrl.Consequent(np.arange(0.0, 1.0, 0.01), 'control_authority')

# # Generate fuzzy membership functions
# arf['small'] = fuzz.trapmf(arf.universe, [1.0, 1.0, 1.5, 2.5])
# arf['medium'] = fuzz.trimf(arf.universe, [1.5, 2.5, 3.5])
# arf['large'] = fuzz.trapmf(arf.universe, [2.5, 2.5, 5.0, 5.0])

# steer_dev['very small'] = fuzz.trapmf(steer_dev.universe, [0.0, 0.0, 0.075, 0.15])
# steer_dev['small'] = fuzz.trimf(steer_dev.universe, [0.075, 0.15, 0.25])
# steer_dev['medium'] = fuzz.trimf(steer_dev.universe, [0.15, 0.25, 0.35])
# steer_dev['large'] = fuzz.trapmf(steer_dev.universe, [0.25, 0.35, 0.5, 0.5])

# control_authority['low'] = fuzz.trapmf(control_authority.universe, [0.0, 0.0, 0.2, 0.4])
# control_authority['medium'] = fuzz.trimf(control_authority.universe, [0.2, 0.4, 0.6])
# control_authority['high'] = fuzz.trimf(control_authority.universe, [0.4, 0.6, 0.8])
# control_authority['very high'] = fuzz.trapmf(control_authority.universe, [0.7, 0.8, 1.0, 1.0])

# rule1 = ctrl.Rule(arf['small'] & steer_dev['very small'], control_authority['very high'])
# rule2 = ctrl.Rule(arf['small'] & steer_dev['small'], control_authority['high'])
# rule3 = ctrl.Rule(arf['small'] & steer_dev['medium'], control_authority['medium'])
# rule4 = ctrl.Rule(arf['small'] & steer_dev['large'], control_authority['medium'])

# rule5 = ctrl.Rule(arf['medium'] & steer_dev['very small'], control_authority['high'])
# rule6 = ctrl.Rule(arf['medium'] & steer_dev['small'], control_authority['medium'])
# rule7 = ctrl.Rule(arf['medium'] & steer_dev['medium'], control_authority['medium'])
# rule8 = ctrl.Rule(arf['medium'] & steer_dev['large'], control_authority['low'])

# rule9 = ctrl.Rule(arf['large'] & steer_dev['very small'], control_authority['medium'])
# rule10 = ctrl.Rule(arf['large'] & steer_dev['small'], control_authority['low'])
# rule11 = ctrl.Rule(arf['large'] & steer_dev['medium'], control_authority['low'])
# rule12 = ctrl.Rule(arf['large'] & steer_dev['large'], control_authority['low'])

# rule_list = [rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12]
# control_authorityping_ctrl = ctrl.ControlSystem(rule_list)
# control_authorityping = ctrl.ControlSystemSimulation(control_authorityping_ctrl)

# control_authorityping.input['arf'] = 5.0#np.arange(1.0, 5.0, 0.5)#6.5
# control_authorityping.input['steer_dev'] = 0.5#np.arange(0.0, 0.5, 0.0625)#9.8