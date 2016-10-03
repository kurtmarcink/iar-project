from wall import Wall


class Robot:
    def __init__(self, conn):
        self.conn = conn
        self.following_wall = Wall.NONE
        self.ir_result = None
        self.turning_to_evade = False
