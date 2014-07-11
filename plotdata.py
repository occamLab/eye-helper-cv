import matplotlib.pyplot as plt  
import numpy as np 
import pickle

#graphs we're interested in:
# speed v. accuracy
#   rotation, affine, overall


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

if __name__ == '__main__':
    temp = open('./OT-res/pickles/p0/catfood.p', 'r')
    data = pickle.load(temp)
    print data
    #plot(data = , plot_type = , plot_xlabel = , plot_ylabel = , plot_title = )