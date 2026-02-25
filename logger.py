class Logger:
    def __init__(self,):
        pass

    def log(self,):
        pass

    def log_step(self, *args, **kwargs):
        # the airspace state will be used for plotting the vertiports
        # the atc_state will be used for UAV positions for plotting 
        pass

    def get_simulator_start_metrics(self,):
        # use database to calculate the start metrics 
        # the database will be used to fetch data needed for calculating the start metrics
        pass

    def get_simulator_end_metrics(self,):
        # use database to calculate the end metrics 
        # the database will be used to fetch data needed for calculating the end metrics
        pass


    def reset(self,):
        # create new database to store states
        pass

    def get_step_render_data(self,):
        # for each step of the simulator get the data needed for rendering that step
        pass
