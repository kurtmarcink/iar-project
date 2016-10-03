class Robot:
    def __init__(self, conn):
        self.conn = conn
        self.following_wall = False
        self.ir_result = None
