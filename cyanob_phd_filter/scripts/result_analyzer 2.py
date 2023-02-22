#!/usr/bin/env python
from re import I
from xml.etree.ElementTree import tostring
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import rospy
import os
#from time_update import *

class analyze():

    # tu=time_update.readfile(os.path.expanduser("~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/test1/coordinates.txt"))
    # plot=time_update.plot(time_update.df,time_update.df)

    def readfile(self,filepath):
        #if file not found, exit
        #print(filepath)
        if os.path.isfile(filepath) == False:
            print("File not found..!!!",filepath)
            print(filepath)
            raise OSError()
            exit()
        
        df =pd .read_csv(filepath,delim_whitespace=True,header=None,names=['x','y','w'],skiprows=1,skipfooter=2)
        #print(self.df)
        if df.empty:
            print("empty file")
            print(filepath)
            exit()
        return df

    # coordinates duplicated by the associated weight 
    # all the weight is changed to one   
    def normalize_weight(self,df):
        df['w']=df['w'].apply(np.ceil) #rounding up the weights

        df_greaterthan1=df.loc[df.w>1]
        df.loc[df.w>1,'w']=1

        for idx in df_greaterthan1.index:
            #print(idx)
            #print(self.df.loc[idx],len(self.df.index))
            for i in range(int(df_greaterthan1.at[idx,'w'])-1):
                df.loc[len(df.index)]=df.loc[idx]
        return df

    def save(self,df,path):#saving after changing weight to 1
        print('saving text')
        newpath=path.replace('_mu','__mu_AllWIs01') #saving file will have __mu_AllWIs01 in its name.
        df.to_csv(path_or_buf=newpath,index=False, sep=" ")



    def plot(self):
        ax1=self.GT.plot.scatter(x='x',y='y', color="blue", label="Ground truth",xlim=[-200,200]) # plotting ground truth 
        #self.TU.plot.scatter(x='x',y='y', color="Green", label="time update",ax=ax1,alpha=0.3)#plotting time update
        self.MU.plot.scatter(x='x',y='y', color="red", label="Estimates",ax=ax1,marker='+')#plotting measurement update
        
        plt.title('Estimates (Time step '+str(i)+')')
        plt.xlabel('X-Axis (meters)')
        plt.ylabel('Y-Axis (meters)')

        plt.axis([-160,160,-110,110])
        plt.grid()
        plt.savefig(self.dirName+self.figname+'.png',format="png")
        plt.savefig(self.dirName+self.figname+'.pdf',format="pdf")
        plt.show()

        #plt.pause(1.00)
        #input("Press Enter to continue...")
        #plt.close()

    def __init__(self, fileGT,fileTU,fileMU):
        self.resultsfolderpath=(os.path.expanduser("~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/NEWindReflect/3/"))#"~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/test_f10_t30_n14/"))
        self.groundtruthfolderpath=(os.path.expanduser("~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/1.groundtruth_blocations/bloom_coordinates_northeast_wind_drift_with_noise_and_reflection/"))
        
        self.dirName=self.resultsfolderpath+'/graphs'
        if not os.path.exists(self.dirName):
            os.mkdir(self.dirName)
            print("Directory " , self.dirName ,  " Created ")
        else:    
            print("Directory " , self.dirName ,  " already exists")
        
        self.dirName=self.dirName+'/'
        self.GT=self.readfile(self.groundtruthfolderpath+fileGT)
        self.TU=self.readfile(self.resultsfolderpath+fileTU)
        self.MU=self.readfile(self.resultsfolderpath+fileMU)

        #In MU file, all coordinates duplicated by the associated weight value. after that weight changed to one.
        self.MuModified=self.normalize_weight(self.MU)
        self.save(self.MuModified,(self.dirName+fileMU)) #file is saved in the folder. __mu_AllWIs01 added to the title

        self.figname='Time_steps_'+str(i)
        self.plot()

        

def main():
    number_of_filesets=10

    global i
    for i in range(number_of_filesets):
        print(i)
        analyze('bloom_coordinates_northeast_wind_drift_with_noise_and_reflection'+str(i)+'.txt','WOIPinitial_ground_truth-'+str(i+1)+'.txt','initial_ground_truth_mu-'+str(i+1)+'.txt')


if __name__ == '__main__':
    main()