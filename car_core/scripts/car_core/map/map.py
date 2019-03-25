# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.


class Map:
    """
    Provide access to the digital environment map built
    by computer vision system
    """
    def __init__(self):
        pass

    def is_obstacle(self, t, pos):
        """
        Checks is there an obstacle. Allow checking in future using
        extrapolated data
        Args:
            t: time
            pos: Position (x,y) in global frame

        Returns: Ts there obstacle or not

        """
        return False
