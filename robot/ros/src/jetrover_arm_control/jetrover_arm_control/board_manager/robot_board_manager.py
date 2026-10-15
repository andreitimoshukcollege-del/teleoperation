import logging

from datetime import datetime, timezone

from jetrover_arm_control.board_manager.robot_controller_sdk import Board

class BoardManager():

    def __init__(self):

        self._board = None

        self.uartAddress = "/dev/ttyACM0"

        if self._board == None:
            self._initializeBoard()
            logging.info("Board is set to Active state")
        else:
            logging.warning("Board is already active")

    def _initializeBoard(self):
        try:
            self._board = Board(device= self.uartAddress)
            self._activateBoard()
            
            logging.info("Board initialized successfully.")
        except Exception as e:
            logging.error(f"Failed to initialize board: {e}")
        


    def _activateBoard(self):
        
        self._board.enable_reception()
        logging.info("Robot Controller Initialized.")

    def getBoard(self):
        if self._board is not None:
            return self._board
        else:
            return False

    

        
        