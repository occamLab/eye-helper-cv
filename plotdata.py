import matplotlib.pyplot as plt  

def plot(data, plot_type, plot_xlabel, plot_ylabel, plot_title):
    """Inputs:
            data -> in a list [x, y]
            plot type -> string either 'scatter' or 'histogram'
            xalabel -> string title of the x axis
            yalabel -> string title of the y axis
            plot_title -> string of the title of the plot
        
        Outputs:
            Plots the data (no actual return)
    """
    plt.plot(data[0], data[1])
    plt.ylabel(plot_ylabel)
    plt.xlabel(plot_xlabel)
    plt.title(plot_title)
    plt.show()    