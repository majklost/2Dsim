import os
CUR_DIR =  os.path.dirname(os.path.abspath(__file__))
FOLDER_PATH = os.path.join(CUR_DIR, "debug/")

class DebugManager:
    """Manges debug folder and debug images"""

    def clear(self):
        """Clears the debug folder"""
        for file in os.listdir(FOLDER_PATH):
            os.remove(FOLDER_PATH + file)











if __name__ == "__main__":
    dm = DebugManager()
    dm.clear()
    print("Debug folder cleared")
