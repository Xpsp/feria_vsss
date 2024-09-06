import rospkg, numpy as np

Xbox360 = {'buttons': ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START', 'LSB', 'RSB'],
           'axes': ['LH', 'LV', 'LT', 'RH', 'RV', 'RT', 'PADH', 'PADV']}

XboxOne = {'buttons': ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START', 'XBOX', 'LSB', 'RSB'],
           'axes': ['LH', 'LV', 'LT', 'RH', 'RV', 'RT', 'PADH', 'PADV']}

Machenike = {'buttons': ['B', 'A', None, 'Y', 'X', None, 'LB', 'RB', 'RTB', 'RTB', 'BACK', 'START', None, 'LSB', 'RSB', None],
           'axes': ['LH', 'LV', 'RH', 'RV', 'RT', 'LT', 'PADH', 'PADV']}

np.savez(rospkg.RosPack().get_path('drlarac_vsss')+'/scripts/modules/joysticks.npz', Xbox360=Xbox360, XboxOne=XboxOne, Machenike=Machenike)