from typing import List, Tuple

# Robot Positions
HOME_POSITION   = [3.863, -1.765, 1.983, -1.760, -1.572, 0.720]
PLACE_POSITION  = [1.916, -1.487, 2.453, -2.564, -1.555, 3.480]
HOLD_POSITION   = [1.086, -1.350, 1.899, -2.128, -1.541, -3.613]
MOUNT_POSITION  = [1.086, -1.350, 1.899, -2.128, -1.541, -3.613]

class Object:

    def __init__(self, name: str, pick_position: List[float], place_position: List[float] = PLACE_POSITION):

        # Object Name, Pick Position, Place Position
        self.name: str = name
        self.pick_position: List[float] = pick_position
        self.place_position: List[float] = place_position

    def getName(self):
        return self.name

    def getPickPosition(self):
        return self.pick_position

    def getPlacePosition(self):
        return self.place_position

available_objects = [
    'screws',
    'planetary gears',
    'sun gear',
    'base',
    'top case',
    'transmission crosses'
]

object_list = [
    Object('screws',                [3.864, -1.401, 2.343, -2.483, -1.571, 0.720]),
    Object('screwdriver',           [3.864, -1.401, 2.343, -2.483, -1.571, 0.720]),
    Object('planetary gears',       [4.184, -1.505, 2.471, -2.508, -1.561, 1.039]),
    Object('sun gear',              [4.002, -1.251, 2.151, -2.472, -1.539, 2.396]),
    Object('base',                  [4.301, -1.418, 2.197, -2.341, -1.597, 0.443]),
    Object('top case',              [4.301, -1.418, 2.197, -2.341, -1.597, 0.443]),
    Object('transmission crosses',  [4.301, -1.418, 2.197, -2.341, -1.597, 0.443]),
]

# Obtain Object Positions
def get_object_positions(object_name: str) -> Tuple[List[float], List[float]]:

    # Return Object Positions if Object found in Object List (by Name) else None
    return next(((obj.getPickPosition(), obj.getPlacePosition) for obj in object_list if obj.getName() == object_name), (None, None))
