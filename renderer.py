



class Renderer:
    def __init__(self,):
        self.data_storeage = None

    def store(self,data):
        '''save state data as JSON'''
        self.data_storeage.save(data) #define a data_storage type which will have some 'save' method 

    