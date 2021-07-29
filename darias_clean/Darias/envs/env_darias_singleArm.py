import os


class env_darias_singleArm():

    def __init__(self):

        self.filePath=os.path.dirname(__file__)
        self.robotFolder=os.path.join(self.filePath,'/../model')
        print(self.robotFolder)

if __name__ == '__main__':
    a=env_darias_singleArm()