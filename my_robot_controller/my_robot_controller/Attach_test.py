from robolink import *    # RoboDK's API
from robodk import *      # Math toolbox for robots

RDK = Robolink()

robot = RDK.Item('', ITEM_TYPE_ROBOT)


target = RDK.Item('Target 7')
item = RDK.item('Pod Tray')

robot.MoveJ(target)