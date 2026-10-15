
from jetrover_arm_control.board_manager.robot_board_manager import BoardManager

class RobotVal:

    def __init__(self, boardMgr: BoardManager):
        
        self.boardMgr = boardMgr
        self.board = self.getBoard()

    def getBattery(self):
        data = self.board.get_battery()

        return data
    
    def setBuzzerCbk(self, msg):
        # /robot/buzzer (std_msgs/Bool): true plays one short beep, false does nothing.
        # The subscription passes the message; without this parameter every message raised a
        # TypeError that killed the whole driver (2026-10-14, robot/README.md). This cannot silence
        # the board's low-battery alarm: that is the board firmware's, and no command turns it off.
        if msg.data:
            self.board.set_buzzer(3000, 0.05, 0.01, 1)
    
    def getBoard(self):
        return self.boardMgr.getBoard()

    