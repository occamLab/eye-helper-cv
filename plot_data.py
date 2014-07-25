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
    plt.plot(data[0], data[1], 'o', label = label)

    plt.ylabel(plot_ylabel)
    plt.xlabel(plot_xlabel)
    plt.title(plot_title)

if __name__ == '__main__':
    pickle_list = ['catfood', 'catfood-a-long', 'catfood-a-short', 'catfood-r-long', 'catfood-r-short','cereal', 'cereal-a-long', 'cereal-a-short', 'cookie', 'cookie-a-long', 'cookie-a-short']
    for thing in pickle_list:
        try:
            name = '../OT-res/pickles/p3/%s.p' %(thing)
            print(name)
            temp = open(name, 'r')
            data = pickle.load(temp)
            for key in data:
                plt.hold(True) 
                x = data[key][4]['start_to_end_time'] # avg runtime per frame for the current method
                y = data[key][2] # avg success rate for the current method  
                plot(data = [x,y],  
                     plot_xlabel = 'average runtime per frame (seconds)',
                     plot_ylabel = 'average success rate (% success)', 
                     plot_title = thing,
                     label = str(key))
            plt.legend()
            plt.savefig("../OT-res/plots/p3/%s.png" % thing)
            plt.show()
        except:
            print('hello')
            pass
