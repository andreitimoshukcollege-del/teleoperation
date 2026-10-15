
from jetrover_arm_control.board_manager.robot_board_manager import BoardManager

class RobotVal:

    def __init__(self, boardMgr: BoardManager):
        
        self.boardMgr = boardMgr
        self.board = self.getBoard()

    def getBattery(self):
        data = self.board.get_battery()

        return data
    
    def setBuzzerCbk(self):
        self.board.set_buzzer(3000, 0.05, 0.01, 1)
    
    def getBoard(self):
        return self.boardMgr.getBoard()

    