class Robot:
    def __init__(self, conn):
        self.conn = conn
        self.following_wall = 0
        self.ir_result = None
        self.turning_to_evade = False
