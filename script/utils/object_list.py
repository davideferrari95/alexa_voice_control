from typing import List, Tuple

# Robot Positions
HOME_POSITION   = [-1.4263079802142542, -1.6319390736021937, -2.4856200218200684, -0.5421183866313477, 1.5817430019378662, 0.15075159072875977]
PLACE_POSITION  = [-0.7629254500018519, -1.8036452732481898, -2.3394904136657715, -0.5377047818950196, 1.5661885738372803, 0.844707727432251]
HOLD_POSITION   = [-1.208057705556051, -1.5532835286906739, -1.907097339630127, -2.8095599613585414, -1.1707270781146448, -0.015118900929586232]
MOUNT_POSITION  = [-1.208033863698141, -1.5533316892436524, -1.9073967933654785, -2.810709615746969, -1.1707032362567347, -1.6355813185321253]

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
    Object('screws',                [0.09259986877441406, -1.3229249280742188, -2.8120217323303223, -0.5384486478618165, 1.5912954807281494, 0.4410998821258545]),
    Object('screwdriver',           [0.5241189002990723, -1.2935460370830079, -2.8341526985168457, -0.6099613469890137, 1.6300556659698486, 0.5266001224517822]),
    Object('base',                  [-1.607004467641012, -1.6019226513304652, -2.5795512199401855, -0.5424302381328125, 1.5807373523712158, 0.15044021606445312]),
    # Object('top case',              [4.301, -1.418, 2.197, -2.341, -1.597, 0.443]),
    # Object('planetary gears',       [4.184, -1.505, 2.471, -2.508, -1.561, 1.039]),
    # Object('sun gear',              [4.002, -1.251, 2.151, -2.472, -1.539, 2.396]),
    # Object('transmission crosses',  [4.301, -1.418, 2.197, -2.341, -1.597, 0.443]),
]

# Obtain Object Positions
def get_object_positions(object_name: str) -> Tuple[List[float], List[float]]:

    # Return Object Positions if Object found in Object List (by Name) else None
    return next(((obj.getPickPosition(), obj.getPlacePosition()) for obj in object_list if obj.getName() == object_name), (None, None))
