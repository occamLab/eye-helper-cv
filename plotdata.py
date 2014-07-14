import matplotlib.pyplot as plt  
import numpy as np 
import pickle

#graphs we're interested in:
# speed v. accuracy
#   rotation, affine, overall
#temporal distance from training image


def plot(data, plot_xlabel, plot_ylabel, plot_title, label):
    """Inputs:
            data -> in a list [x, y]
            plot type -> string either 'scatter' or 'histogram'
            xalabel -> string title of the x axis
            yalabel -> string title of the y axis
            plot_title -> string of the title of the plot
            label -> What is this data of
        
        Outputs:
            Plots the data (no actual return)
    """
    if len(data) > 1:
        plt.plot(data[0], data[1], label = label)
    else:
        plt.plot(data, label = label)

    plt.ylabel(plot_ylabel)
    plt.xlabel(plot_xlabel)
    plt.title(plot_title)

if __name__ == '__main__':
    pickle_list = ['catfood', 'catfood-a-long', 'catfood-a-short', 'catfood-r-long', 'catfood-r-short','cereal', 'cereal-a-long', 'cereal-a-short', 'cookie', 'cookie-a-long', 'cookie-a-short']
    for pickle in pickle_list:
        try:
            temp = open('./OT-res/pickles/p2/%s' %(pickle), 'r')
            data = pickle.load(temp)
            print data
            for key in data:
                x = data[key][3]
                y = data[key][2]
                plt.hold(True) 
                plot(data = [x,y], plot_xlabel = 'time of run (s)', 
                            plot_ylabel = 'Percent accuracy', 
                            plot_title = 'frames from training image',
                            label = pickle + ' '+ str(key))
            plt.legend()
        except:
            pass
    plt.show()
